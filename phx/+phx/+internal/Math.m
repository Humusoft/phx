classdef Math
%phx.internal.Math Internal mathematical functions

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    methods (Static)
        % Transformation matrix for rotation around any axis
        function M = rotAA(Axis, Angle)
            if any(Axis(:))
                Axis = Axis/norm(Axis);
                cB = cos(Angle);
                sB = sin(Angle);
                M = zeros(3);

                M(1, 1) = cB + Axis(1)*Axis(1)*(1 - cB);
                M(1, 2) = Axis(1)*Axis(2)*(1 - cB) - Axis(3)*sB;
                M(1, 3) = Axis(1)*Axis(3)*(1 - cB) + Axis(2)*sB;

                M(2, 1) = Axis(1)*Axis(2)*(1 - cB) + Axis(3)*sB;
                M(2, 2) = cB + Axis(2)*Axis(2)*(1 - cB);
                M(2, 3) = Axis(2)*Axis(3)*(1 - cB) - Axis(1)*sB;

                M(3, 1) = Axis(1)*Axis(3)*(1 - cB) - Axis(2)*sB;
                M(3, 2) = Axis(2)*Axis(3)*(1 - cB) + Axis(1)*sB;
                M(3, 3) = cB + Axis(3)*Axis(3)*(1 - cB);
            else
                M = eye(3);
            end
        end

        % Transformation matrix for rotations in order Z -> Y -> X
        function M = rot321(XYZ)
            c = cos(XYZ);
            s = sin(XYZ);

            X = [1 0 0; 0 c(1) -s(1); 0 s(1) c(1)];
            Y = [c(2) 0 s(2); 0 1 0; -s(2) 0 c(2)];
            Z = [c(3) -s(3) 0; s(3) c(3) 0; 0 0 1];
            M = Z*Y*X;
        end

        % Decomposition of transformation matrix for rotations in order Z -> Y -> X
        function xyz = decomp321(R)
            zyx = robotics.internal.rotm2eul(R, "ZYX");
            xyz = zyx([3 2 1]);
        end

        % Decomposition of transformation matrix for axis-angle rotation
        function aa = decompAA(R)
            aa = robotics.internal.rotm2axang(R);
        end
    end

end