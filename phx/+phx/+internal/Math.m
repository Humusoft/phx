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
            sy = min(max(-R(3, 1), -1), 1);

            if abs(sy) < 1 - 1e-10
                xyz = [atan2(R(3, 2), R(3, 3)), asin(sy), atan2(R(2, 1), R(1, 1))];
            else
                % gimbal lock (Y = +/-pi/2): X and Z are coupled, choose X = 0
                xyz = [0, asin(sy), atan2(-R(1, 2), R(2, 2))];
            end
        end

        % Transformation matrix for quaternion rotation [w x y z]
        function M = rotQ(Q)
            Q = Q/norm(Q);
            w = Q(1); x = Q(2); y = Q(3); z = Q(4);

            M = [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y); ...
                 2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x); ...
                 2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)];
        end

        % Decomposition of transformation matrix for quaternion rotation [w x y z]
        function q = decompQ(R)
            % Shepperd's method: pivot on the largest of trace and diagonal
            % elements so the square root argument stays well away from zero
            [~, i] = max([trace(R), R(1, 1), R(2, 2), R(3, 3)]);
            switch i
                case 1
                    r = sqrt(1 + R(1, 1) + R(2, 2) + R(3, 3));
                    s = 0.5/r;
                    q = [0.5*r, (R(3, 2) - R(2, 3))*s, (R(1, 3) - R(3, 1))*s, (R(2, 1) - R(1, 2))*s];
                case 2
                    r = sqrt(1 + R(1, 1) - R(2, 2) - R(3, 3));
                    s = 0.5/r;
                    q = [(R(3, 2) - R(2, 3))*s, 0.5*r, (R(1, 2) + R(2, 1))*s, (R(1, 3) + R(3, 1))*s];
                case 3
                    r = sqrt(1 - R(1, 1) + R(2, 2) - R(3, 3));
                    s = 0.5/r;
                    q = [(R(1, 3) - R(3, 1))*s, (R(1, 2) + R(2, 1))*s, 0.5*r, (R(2, 3) + R(3, 2))*s];
                case 4
                    r = sqrt(1 - R(1, 1) - R(2, 2) + R(3, 3));
                    s = 0.5/r;
                    q = [(R(2, 1) - R(1, 2))*s, (R(1, 3) + R(3, 1))*s, (R(2, 3) + R(3, 2))*s, 0.5*r];
            end

            if q(1) < 0
                q = -q;
            end
        end

        % Decomposition of transformation matrix for axis-angle rotation
        function aa = decompAA(R)
            q = phx.internal.Math.decompQ(R);
            sinHalf = norm(q(2:4));

            if sinHalf > 1e-12
                aa = [q(2:4)/sinHalf, 2*atan2(sinHalf, q(1))];
            else
                aa = [0 0 1 0];
            end
        end
    end

end