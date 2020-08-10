
function [centers, r_estimated, metric] = imfindcircles(varargin)
%IMFINDCIRCLES Find circles using Circular Hough Transform.
%   CENTERS = IMFINDCIRCLES(A, RADIUS) finds circles with approximately the
%   specified RADIUS, in pixels, in the input image A. A can be a
%   grayscale, RGB or binary image. CENTERS is a P-by-2 matrix with
%   X-coordinates of the circle centers in the first column and the
%   Y-coordinates in the second column. CENTERS is sorted based on the
%   circle strengths.
%
%   [CENTERS, RADII] = IMFINDCIRCLES(A, RADIUS_RANGE) finds circles with
%   radii in the search range specified by RADIUS_RANGE. RADIUS_RANGE is a
%   two-element vector [MIN_RADIUS MAX_RADIUS], where MIN_RADIUS and
%   MAX_RADIUS have integer values. The estimated radii, in pixels, for the
%   circles are returned in the column vector RADII.
%
%   [CENTERS, RADII, METRIC] = IMFINDCIRCLES(A, RADIUS_RANGE) also
%   returns the magnitude of the accumulator array peak associated with
%   each circle in the column vector METRIC. CENTERS and RADII are
%   sorted in descending order of their corresponding METRIC values.
% 
%   [CENTERS, RADII, METRIC] = IMFINDCIRCLES(...,PARAM1,VAL1,PARAM2,VAL2,...)
%   finds circles using name-value pairs to control aspects of the Circular 
%   Hough Transform. Parameter names can be abbreviated.
%
%   Parameters include:
%
%   'ObjectPolarity' - Specifies the polarity of the circular object with
%                      respect to the background. Available options are:
%
%           'bright'     : The object is brighter than the background. (Default)
%           'dark'       : The object is darker than the background.
% 
%   'Method' - Specifies the technique used for computing the accumulator
%              array. Available options are:
%
%           'PhaseCode'  : Atherton and Kerbyson's Phase Coding method.
%                         (Default)
%           'TwoStage'   : The method used in Two-stage Circular Hough
%                          Transform.
% 
%   'Sensitivity '  - Specifies the sensitivity factor in the range [0 1]
%                     for finding circles. A high sensitivity value leads
%                     to detecting more circles, including weak or
%                     partially obscured ones, at the risk of a higher
%                     false detection rate. Default value: 0.85
%
%   'EdgeThreshold' - A scalar K in the range [0 1], specifying the gradient 
%                     threshold for determining edge pixels. K = 0 sets the
%                     threshold at zero-gradient magnitude, and K = 1 sets
%                     the threshold at the maximum gradient magnitude in
%                     the image. A high EdgeThreshold value leads to
%                     detecting only those circles that have relatively
%                     strong edges. A low EdgeThreshold value will, in
%                     addition, lead to detecting circles with relatively
%                     faint edges. By default, IMFINDCIRCLES chooses the
%                     value automatically using the function GRAYTHRESH.
%
%   Notes
%   -----
%   1.  Binary images (must be logical matrix) undergo additional pre-processing 
%       to improve the accuracy of the result. RGB images are converted to
%       grayscale using RGB2GRAY before they are processed.
%   2.  For high accuracy, a relatively small radius range should be used.  
%       Large radius ranges reduce algorithm accuracy and increase
%       computation time. A good rule of thumb is to choose the radius
%       range such that Rmax < 3*Rmin and (Rmax - Rmin) < 100.
%   3.  Accuracy is limited for very small radius values, e.g. Rmin < 10.   
%   4.  The radius estimation step for Phase Coding method is typically
%       faster than that of the Two-Stage method.
%   5.  Both Phase Coding and Two-Stage methods in IMFINDCIRCLES are limited 
%       in their ability to detect concentric circles. The results for
%       concentric circles may vary based on the input image.
%   6.  IMFINDCIRCLES does not find circles with centers outside the image
%       domain.
%
%   Class Support
%   -------------
%   Input image A can be uint8, uint16, int16, double, single or logical,
%   and must be nonsparse. The output variables CENTERS, RADII, and METRIC
%   are of class double.
%
%   Example 1
%   ---------
%   This example finds and displays the five strongest circles in the image
%   coins.png
%
%         A = imread('coins.png');
%         % Display the original image
%         imshow(A)
% 
%         % Find all the circles with radius >= 15 and radius <= 30
%         [centers, radii, metric] = imfindcircles(A,[15 30]);
% 
%         % Retain the five strongest circles according to metric values
%         centersStrong5 = centers(1:5,:);
%         radiiStrong5 = radii(1:5);
%         metricStrong5 = metric(1:5);
% 
%         % Draw the circle perimeter for the five strongest circles
%         viscircles(centersStrong5, radiiStrong5,'EdgeColor','b');
%
%   Example 2
%   ---------
%   This example finds both bright and dark circles in the image
%
%         I = imread('circlesBrightDark.png');
%         imshow(I)
% 
%         Rmin = 30;
%         Rmax = 65;
% 
%         % Find all the bright circles in the image
%         [centersBright, radiiBright] = imfindcircles(I,[Rmin Rmax], ...
%                                       'ObjectPolarity','bright');
% 
%         % Find all the dark circles in the image
%         [centersDark, radiiDark] = imfindcircles(I, [Rmin Rmax], ...
%                                       'ObjectPolarity','dark');
% 
%         % Plot bright circles in blue
%         viscircles(centersBright, radiiBright,'EdgeColor','b');
% 
%         % Plot dark circles in dashed red boundaries
%         viscircles(centersDark, radiiDark,'LineStyle','--');
%
% See also HOUGH, HOUGHPEAKS, HOUGHLINES, VISCIRCLES.

%   Copyright 2011 The MathWorks, Inc.
%   $Revision: 1.1.6.5 $  $Date: 2012/04/20 19:09:48 $

%   References:
%   -----------
%   [1] H. K. Yuen, J. Princen, J. Illingworth, and J. Kittler,
%       "Comparative study of Hough Transform methods for circle finding,"
%       Image and Vision Computing, Volume 8, Number 1, 1990, pp. 71?77.
%
%   [2] E. R. Davies, Machine Vision: Theory, Algorithms, Practicalities -
%       Chapter 10, 3rd Edition, Morgan Kauffman Publishers, 2005.
%
%   [3] T. J. Atherton, D. J. Kerbyson,?"Size invariant circle detection,"
%   ?   Image and Vision Computing, Volume 17, Number 11, 1999, pp. 795-803.  

parsedInputs  = parseInputs(varargin{:});

A             = parsedInputs.Image;
radiusRange   = parsedInputs.RadiusRange; 
method        = lower(parsedInputs.Method);
objPolarity   = lower(parsedInputs.ObjectPolarity);
edgeThresh    = parsedInputs.EdgeThreshold;
sensitivity   = parsedInputs.Sensitivity;

centers = [];
r_estimated = [];
metric = [];

%% Warn if the radius range is too large
if (numel(radiusRange) == 2)
    if ((radiusRange(2) > 3*radiusRange(1)) || ((radiusRange(2)-radiusRange(1)) > 100))
        warning(message('images:imfindcircles:warnForLargeRadiusRange', upper(mfilename), ...
                'Rmax < 3*Rmin','(Rmax - Rmin) < 100','[20 100]',upper(mfilename),...
                sprintf('\t[CENTERS1, RADII1, METRIC1] = IMFINDCIRCLES(A, [20 60]);\n\t[CENTERS2, RADII2, METRIC2] = IMFINDCIRCLES(A, [61 100]);')))
    end
end

%% Warn if the minimum radius is too small
if (radiusRange(1) <= 5)
    warning(message('images:imfindcircles:warnForSmallRadius', upper(mfilename)))
end

%% Compute the accumulator array
[accumMatrix, gradientImg] = chaccum(A, radiusRange, 'Method',method,'ObjectPolarity', ...
                        objPolarity,'EdgeThreshold',edgeThresh);

%% Check if the accumulator array is all-zero
if (~any(accumMatrix(:)))
    return;
end                    

%% Estimate the centers
accumThresh = 1 - sensitivity;
[centers, metric] = chcenters(accumMatrix, accumThresh);

if (isempty(centers)) % If no centers are found, no further processing is necessary
    return;
end

%% Retain circles with metric value greater than threshold corresponding to AccumulatorThreshold 
idx2Keep = find(metric >= accumThresh);
centers = centers(idx2Keep,:);
metric = metric(idx2Keep,:);

if (isempty(centers)) % If no centers are retained, no further processing is necessary
    centers = []; % Make it 0x0 empty
    metric = [];
    return;
end

%% Estimate radii
if (nargout > 1)
    if (length(radiusRange) == 1)
        r_estimated = repmat(radiusRange,size(centers,1),1);
    else
        switch (method)
            case 'phasecode'
                r_estimated = chradiiphcode(centers, accumMatrix, radiusRange);                
            case 'twostage'
                r_estimated = chradii(centers, gradientImg, radiusRange);
            otherwise
                iptassert(false,'images:imfindcircles:unrecognizedMethod'); % Should never happen
        end
    end    
end
end


function parsedInputs = parseInputs(varargin)

narginchk(2,Inf);

persistent parser;

if (isempty(parser))
    parser = inputParser();

    parser.addRequired('Image',@checkImage);
    parser.addRequired('RadiusRange',@checkRadiusRange);
    parser.addParamValue('Method','phasecode',@checkMethod);
    parser.addParamValue('ObjectPolarity','bright');
    parser.addParamValue('EdgeThreshold',[]);
    parser.addParamValue('Sensitivity',0.85,@checkSensitivity);
end

% Parse input, replacing partial name matches with the canonical form.
if (nargin > 2) % If any name-value pairs are given
  varargin(3:end) = remapPartialParamNames({'Method', 'ObjectPolarity', ...
                                            'EdgeThreshold', 'Sensitivity'}, ...
                                            varargin{3:end});
end

parser.parse(varargin{:});
parsedInputs = parser.Results;

validateRadiusRange(); % If Rmin and Rmax are the same then set R = Rmin.
    
    function tf = checkImage(A)
        allowedImageTypes = {'uint8', 'uint16', 'double', 'logical', 'single', 'int16'};
        validateattributes(A,allowedImageTypes,{'nonempty',...
            'nonsparse','real'},mfilename,'A',1);
        N = ndims(A);
        if (isvector(A) || N > 3)
            error(message('images:imfindcircles:invalidInputImage'));
        elseif (N == 3)
            if (size(A,3) ~= 3)
                error(message('images:imfindcircles:invalidImageFormat'));
            end
        end
        tf = true;
    end

    function tf = checkRadiusRange(radiusRange)
        if (isscalar(radiusRange))
            validateattributes(radiusRange,{'numeric'},{'nonnan', ...
                'nonsparse','nonempty','positive','finite','vector'},mfilename,'RADIUS_RANGE',2);
        else
            validateattributes(radiusRange,{'numeric'},{'integer','nonnan', ...
                'nonsparse','nonempty','positive','finite','vector'},mfilename,'RADIUS_RANGE',2);
        end
        if (length(radiusRange) > 2)
            error(message('images:imfindcircles:unrecognizedRadiusRange'));
        elseif (length(radiusRange) == 2)
            if (radiusRange(1) > radiusRange(2))
                error(message('images:imfindcircles:invalidRadiusRange'));
            end
        end
        
        tf = true;
    end

    function tf = checkMethod(method)
        validatestring(lower(method), {'twostage','phasecode'}, ...
            mfilename, 'Method');
        
        tf = true;
    end   

    function tf = checkSensitivity(s)
        validateattributes(s,{'numeric'},{'nonempty','nonnan', ...
            'finite','scalar'},mfilename,'Sensitivity');
        if (s > 1 || s < 0)
            error(message('images:imfindcircles:outOfRangeSensitivity'));
        end
        tf = true;
    end

    function validateRadiusRange
        if (length(parsedInputs.RadiusRange) == 2)
            if (parsedInputs.RadiusRange(1) == parsedInputs.RadiusRange(2))
                parsedInputs.RadiusRange = parsedInputs.RadiusRange(1);
            end
        end
    end
        
end

