function areas = compute_bbox_areas(bboxes)
% COMPUTE_BBOX_AREAS  Compute bottom‐face area for each 8×3 bbox
%
%   areas = compute_bbox_areas(bboxes)
%
% Input:
%   bboxes – K×1 cell array, each an 8×3 array of corners 
%            (rows 1–4 are bottom face in clockwise order)
%
% Output:
%   areas  – K×1 vector of areas (same units as your point‐cloud coords)

    K = numel(bboxes);
    areas = zeros(K,1);

    for k = 1:K
        B = bboxes{k};
        % pick two adjacent bottom‐face edges:
        v1 = B(2,:) - B(1,:);
        v2 = B(4,:) - B(1,:);
        % area = |v1 × v2|  (parallelogram = rectangle)
        areas(k) = norm(cross(v1, v2));
    end
end