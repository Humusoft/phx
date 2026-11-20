function [V, F, N] = computeLiquidSurface(P, d, threads) %#ok<INUSD>
%phx.internal.computeLiquidSurface Surface of a liquid of particles
% P: 3xN array of particle positions
% d: rest spacing of the particles
% threads: threads of the MEX build (default maxNumCompThreads); unused
%    here
% V: 3xK single array of surface vertices
% F: 3xM uint32 array of triangles, one per column (vertex indices,
%    one-based)
% N: 3xK single array of unit vertex normals, pointing out of the liquid
%
% The outputs are in the types a TriangleStrip takes: F(:)' is its
% VertexIndices.
%
% The particles are spread trilinearly onto a grid of cells d/2 aligned
% with the world origin (the surface does not shift as the liquid's
% extent changes), the grid
% is smoothed by a separable Gaussian of 0.6 d, and the surface is the
% level 0.5 of that field normalised so that a liquid at rest reads 1 in
% its interior. isosurface without vertex sharing is welded by the grid
% edge each vertex lies on (the same mesh, a third of the time); the
% normals are the sums of the normals of the faces around, weighted by
% their area.
%
% Each cluster of particles gets a grid of its own: the particles are
% sorted into blocks of 4 d, and blocks that touch (also at an edge or a
% corner) form a cluster. A drop far from the rest of the liquid then
% costs a small grid instead of stretching one grid to it.

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    V = zeros(3, 0, "single");
    F = zeros(3, 0, "uint32");
    N = zeros(3, 0, "single");
    if isempty(P)
        return
    end
    Fd = zeros(0, 3);
    Vd = zeros(3, 0);
    Nd = zeros(3, 0);

    % Clusters of touching blocks
    block = floor((P - min(P, [], 2))/(4*d));
    [blocks, ~, ofParticle] = unique(block', "rows");
    nBlocks = size(blocks, 1);
    if nBlocks == 1
        cluster = 1;
    else
        [a, b] = find(triu(true(nBlocks), 1) & ...
            max(abs(permute(blocks, [1 3 2]) - permute(blocks, [3 1 2])), [], 3) <= 1);
        cluster = conncomp(graph(a, b, [], nBlocks));
    end
    ofParticle = cluster(ofParticle);
    for c = 1:max(ofParticle)
        [Vc, Fc, Nc] = surfaceOf(P(:, ofParticle == c), d);
        Fd = [Fd; Fc + size(Vd, 2)]; %#ok<AGROW>
        Vd = [Vd Vc]; %#ok<AGROW>
        Nd = [Nd Nc]; %#ok<AGROW>
    end
    V = single(Vd);
    F = uint32(Fd');
    N = single(Nd);
end

function [V, F, N] = surfaceOf(P, d)
    % The surface of one cluster of particles
    V = zeros(3, 0);
    N = zeros(3, 0);
    h = d/2;
    sigma = 0.6*d/h;          % in cells
    kr = ceil(2.5*sigma);     % kernel half-width in cells

    % Grid around the particles with room for the kernel
    pad = (kr + 2)*h;
    lo = floor((min(P, [], 2) - pad)/h)*h;   % on the world lattice of cells h
    dims = ceil((max(P, [], 2) + pad - lo)/h)' + 1;

    % Trilinear splat: 8 cells per particle, one accumarray
    u = (P - lo)/h;
    i0 = floor(u);
    f = u - i0;
    i0 = i0 + 1;
    n = size(P, 2);
    idx = zeros(8, n);
    w = zeros(8, n);
    c = 0;
    for dz = 0:1
        wz = (1 - f(3, :))*(1 - dz) + f(3, :)*dz;
        for dy = 0:1
            wy = (1 - f(2, :))*(1 - dy) + f(2, :)*dy;
            for dx = 0:1
                wx = (1 - f(1, :))*(1 - dx) + f(1, :)*dx;
                c = c + 1;
                idx(c, :) = (i0(1, :) + dx) + dims(1)*(i0(2, :) + dy - 1) + dims(1)*dims(2)*(i0(3, :) + dz - 1);
                w(c, :) = wx.*wy.*wz;
            end
        end
    end
    G = reshape(accumarray(idx(:), w(:), [prod(dims) 1]), dims);

    % Separable Gaussian, normalised so that a filled interior reads 1
    k = exp(-0.5*((-kr:kr)/sigma).^2);
    k = k/sum(k);
    G = convn(G, k(:), "same");
    G = convn(G, k, "same");
    G = convn(G, reshape(k, 1, 1, []), "same");
    G = G*(d/h)^3;

    % The level 0.5, its unshared vertices welded by the grid edge they lie
    % on: two whole coordinates, the floor of the fractional one, and its
    % axis. isosurface takes the first dimension as y.
    [F, X] = isosurface(G, 0.5, "noshare");
    if isempty(F)
        F = zeros(0, 3);
        return
    end
    fractional = abs(X - round(X)) > 1e-7;
    b = round(X);
    b(fractional) = floor(X(fractional));
    key = ((b(:, 1)*(dims(1) + 2) + b(:, 2))*(dims(3) + 2) + b(:, 3))*4 + fractional*[1; 2; 3];
    [sortedKeys, order] = sort(key);
    first = [true; diff(sortedKeys) ~= 0];
    id = zeros(size(key));
    id(order) = cumsum(first);
    X = X(order(first), :);
    F = id(F);
    % (Swapping x and y mirrors the mesh, which turns isosurface's inward
    % winding outward)
    V = lo + (X(:, [2 1 3])' - 1)*h;

    % Vertex normals: area-weighted sums of the face normals
    a = V(:, F(:, 1));
    e1 = V(:, F(:, 2)) - a;
    e2 = V(:, F(:, 3)) - a;
    fn = [e1(2, :).*e2(3, :) - e1(3, :).*e2(2, :); e1(3, :).*e2(1, :) - e1(1, :).*e2(3, :); ...
        e1(1, :).*e2(2, :) - e1(2, :).*e2(1, :)];
    m = size(V, 2);
    N = zeros(3, m);
    for axis = 1:3
        N(axis, :) = accumarray(F(:), repmat(fn(axis, :)', 3, 1), [m 1])';
    end
    N = N./max(sqrt(sum(N.^2, 1)), realmin);
end
