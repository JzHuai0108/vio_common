function bbox3D = plane_bbox(pts, plane, xy_trim, z_thickness)
% plane_bbox  Wire‐frame bounding box (8 corners) around pts on a plane
%
% Inputs:
%   pts         Nx3 array of points
%   plane       struct with fields:
%                  .Normal  (1×3) unit normal
%                  .D       scalar so that Normal*X' + D == 0
%   xy_trim     scalar inset of the 2D box (along both +u and +v)
%   z_thickness total thickness along the normal direction
%
% Output:
%   bbox3D      8×3 array of corners:
%               rows 1–4 are the bottom face (clockwise from lower‐left),
%               rows 5–8 are the top face (same ordering)

    % 1) Orthonormal basis (u,v,n)
    n = plane.Normal(:);
    n = n / norm(n);
    if abs(n(1)) < 0.9
        tmp = [1; 0; 0];
    else
        tmp = [0; 1; 0];
    end
    u = cross(n, tmp);  u = u / norm(u);
    v = cross(n, u);    % already unit-length

    % 2) Project points onto the plane
    dists     = pts * n + plane.D;    % Nx1 signed distances
    pts_proj  = pts - dists * n';      % Nx3 projections

    % 3) In‐plane (u,v) coordinates
    x = pts_proj * u;  % Nx1
    y = pts_proj * v;  % Nx1

    % 4) Compute trimmed UV bounds
    xmin = min(x) + xy_trim;
    xmax = max(x) - xy_trim;
    ymin = min(y) + xy_trim;
    ymax = max(y) - xy_trim;
    if xmin > xmax || ymin > ymax
        error('xy_trim too large: box collapsed');
    end

    % 5) 2D box corners in (u,v) frame
    uv2D = [ xmin, ymin;
             xmax, ymin;
             xmax, ymax;
             xmin, ymax ];

    % 6) Find the plane’s “origin” (the closest point to [0,0,0])
    planeOrigin = -plane.D * n;  % 3×1

    % 7) Lift into 3D (bottom & top faces)
    halfZ = z_thickness/2;
    bbox3D = zeros(8,3);
    for i = 1:4
        cornerUV = uv2D(i,1)*u + uv2D(i,2)*v;
        basePt   = planeOrigin + cornerUV;
        bbox3D(i,   :) = (basePt - halfZ * n)';  % bottom face
        bbox3D(i+4, :) = (basePt + halfZ * n)';  % top face
    end
end
