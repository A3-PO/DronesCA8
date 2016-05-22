function varargout = uas_los(Z, R, gs_lat, gs_lon, ua_lat, ua_lon, ...
    gs_alt, ua_alt, gs_altopt, ua_altopt, actualradius, apparentradius)
%
%   In order to work, this function needs to be in the Mapping Toolbox:
%   ..\matlab\toolbox\map\map
%
%   # LOS visibility between GS and UA in terrain with elevation #
%               GS  - Ground Station
%               UA  - Unmanned Aicraft
%               LOS - Line of Sight  
%
%   - Computes the mutual visibility between two points on a displayed
%   digital elevation map   
%   - The two points are selected by clicking on the map 
%   - The result is displayed in a new figure  
%   - Markers indicate visible and obscured points along the profile
%   - The profile is shown in a Cartesian coordinate system with the origin
%   at the GS location
%   - The displayed z coordinate accounts for the elevation of the terrain 
%   and the curvature of the body (Earth).
%
%   VIS = UAS_LOS(Z, R, GS_LAT, GS_LON, UA_LAT, UA_LON, GS_ALT, UA_ALT, ...
%       GS_ALT_OPT, UA_ALT_OPT, ACTUALRADIUS, EFFECTIVERADIUS) 
%
%   Z - Data grid containing elevations in units of meters
%
%   R - Geographic raster reference object, its RasterSize property
%   must be consistent with size(Z)
%
%   If R is a referencing matrix, it must be 3-by-2 and transform raster
%   row and column indices to/from geographic coordinates according to:
% 
%                     [lon lat] = [row col 1] * R.
%
%   If R is a referencing matrix, it must define a (non-rotational,
%   non-skewed) relationship in which each column of the data grid falls
%   along a meridian and each row falls along a parallel.
%
%   GS_LAT - Latitude at which the GS is located (fixed)
%   GS_LON - Longitude at which the GS is located (fixed)
%   UA_LAT - Latitude at which the UA is located at that moment
%   UA_LON - Longitude at which the UA is located at that moment
%   GS_ALT - Altitude at which the GS is placed
%   UA_ALT - Altitude at which the UA is flying above the surface
%   GS_ALT_OPT - Altitude option for GS (def 'AGL')
%   UA_ALT_OPT - Altitude option for UA (def 'AGL')
%   ACTUALRADIUS - Earth Radius in meters
%   EFFECTIVERADIUS - 4/3*ACTUALRADIUS
%
%   ALT_OPT is either:
%   - a relative altitude (ALT_OPT equals 'AGL', the default) interpreted 
%   as the altitude (in meters) above terrain ("Above Ground Level")
%   - an absolute altitude (ALT_OPT equals 'MSL') is interpreted as 
%   altitude above zero ("Mean Sea Level")
%
%   ACTUALRADIUS & EFFECTIVERADIUS
%   Assumes a larger radius for propagation of the LOS. This can account 
%   for the curvature of the signal path due to refraction in the atmosphere.  
%   For example, radio propagation in the atmosphere is commonly treated as
%   straight line propagation on a sphere with 4/3rds the radius of the 
%   earth. In that case the last two arguments would be:
%   >>>R_e (the radius of the earth) and 4/3*R_e
%   
%   For flat earth visibility calculations use Inf as the effective radius
%   The altitudes, elevations and the radii should be in the same units!!! 
%
%   See also LOS2, VIEWSHED

% CA832 AAU Reworked
% Copyright 1996-2014 The MathWorks, Inc.
% Original version (los2) written by Walter Stumpf

if nargin < 2 || (isempty(Z) && isempty(R))
   [Z, R] = getrmm;
end

if nargin < 4 || (isempty(gs_lat) && isempty(gs_lon))
   disp('Click on the map for point 1')
   [gs_lat,gs_lon] = inputm(1);
end

if nargin < 6 || (isempty(ua_lat) && isempty(ua_lon))
   disp('Click on the map for point 2')
   [ua_lat,ua_lon] = inputm(1);
end

if nargin < 7; gs_alt = 100*eps; end   % observer on the surface
if nargin < 8; ua_alt = 0; end         % look at terrain, not above it
if nargin < 9; gs_altopt = 'AGL'; end  % observer altitude above terrain
if nargin < 10; ua_altopt = 'AGL'; end % target above ground level
if nargin < 11; actualradius = earthRadius; end
if nargin < 12; apparentradius = 4/3*actualradius; end 

checklatlon(gs_lat, gs_lon, mfilename, 'GS_LAT', 'GS_LON', 3, 4)
checklatlon(ua_lat, ua_lon, mfilename, 'UA_LAT', 'UA_LON', 5, 6)

if isscalar(gs_alt)
    gs_alt = repmat(gs_alt,size(gs_lat));
end

if isscalar(ua_alt)
    ua_alt = repmat(ua_alt,size(gs_lat));
end

% If R is already spatial referencing object, validate it. Otherwise
% convert the input referencing vector or matrix. And construct a
% non-extrapolating default-grid griddedInterpolant for Z.
R = internal.map.convertToGeoRasterRef( ...
    R, size(Z), 'degrees', mfilename, 'R', 2);
F = griddedInterpolant(Z);
F.ExtrapolationMethod = 'none';

gs_altopt = validatestring(gs_altopt, {'AGL','MSL'}, 'LOS2', 'ALT1OPT', 9);
observerAltitudeIsAGL = strcmp(gs_altopt,'AGL');

ua_altopt = validatestring(ua_altopt, {'AGL','MSL'}, 'LOS2', 'ALT2OPT', 10);
targetAltitudeIsAGL = strcmp(ua_altopt, 'AGL');

% loop over pairs of observer and target locations
makeplot = (nargout == 0);
if numel(gs_lat) == 1
    % Single pair of points:
    %    Output 1 is a logical scalar
    %    Output 2 is a logical array
    %    Outputs 3-6 are numerical arrays
    
    [visprofile, dist, h, lattrk, lontrk, x1, z1, x2, z2] = calculateLOS(F, R, ...
        gs_lat, gs_lon, ua_lat, ua_lon, gs_alt, ua_alt, observerAltitudeIsAGL, ...
        targetAltitudeIsAGL, actualradius, apparentradius);
    
    vis = visprofile(end);
    
    % Display calculation if no output arguments in main function
    if makeplot
        plotProfile(x1, z1, x2, z2, visprofile)
    end
else
    % Multiple pairs of points:
    %   Output 1 is a logical array
    %   Outputs 2-6 are cell arrays that contain
    %      logical or numerical arrays
    vis = false(1, numel(gs_lat));
    visprofile = cell(1, numel(gs_lat));
    h      = cell(1, numel(gs_lat));
    dist   = cell(1, numel(gs_lat));
    lattrk = cell(1, numel(gs_lat));
    lontrk = cell(1, numel(gs_lat));
    
    for i = 1:length(gs_lat)
        [visprofile{i}, dist{i}, h{i}, lattrk{i}, lontrk{i}, x1, z1, x2, z2] ...
            = calculateLOS(F, R, gs_lat(i), gs_lon(i), ua_lat(i), ua_lon(i), ...
                gs_alt(i), ua_alt(i), observerAltitudeIsAGL, ...
                targetAltitudeIsAGL, actualradius, apparentradius);
        
        vis(i) = visprofile{i}(end);
        
        % Display calculation if no output arguments in main function
        if makeplot
            plotProfile(x1, z1, x2, z2, visprofile{i})
        end
    end
end

% Return only the arguments that are requested, if any.
outputArguments = {vis,visprofile,dist,h,lattrk,lontrk};
varargout = outputArguments(1:nargout);   

%-----------------------------------------------------------------------

function plotProfile(x, z, x2, z2, vis)
% Plot the terrain profile plus observer position, visible and obscured
% points, and line of sight to last point in profile, in a new figure with
% the tag 'los2'.

vis = reshape(vis, size(x));

figure('Tag','los2','NextPlot','add')
ax = axes('NextPlot','add');
ax.XTick = 0:10:x(end)
ax.XMinorTick = 'on'
ax.YDir = 'reverse'
ax.YMinorTick = 'on'
h(1) = plot(ax,x/1000,-z,'k');

% Line from beginning to end
% if vis(end)
%     h(5) = plot(ax,[0;x2(end)],[0;z2(end)],'g');
% else
%     h(5) = plot(ax,[0;x2(end)],[0;z2(end)],'r');
% end

if any(vis)
    h(2) = plot(ax,x2(vis)/1000,-z2(vis),'b+');
    h(2).LineWidth = 2;
end

if any(~vis)
    h(3) = plot(ax,x2(~vis)/1000,-z2(~vis),'ro');
    h(3).LineWidth = 2;
end

h(4) = plot(ax,0,0,'x','color',[1 .5 .2]);
h(4).LineWidth = 2.5;
% axis(ax,'equal')

labels = {'Terrain','Visible UA','Unseen UA','Ground Station','Line of Sight'};
indx = ishghandle(h,'line');
leg = legend(ax, h(indx), labels(indx));
set(leg,'FontSize',16);
xlabel(ax,'DISTANCE FROM GS [kilometers]')
ylabel(ax,'DOWN AXIS FROM GS [meters]')
grid minor;
