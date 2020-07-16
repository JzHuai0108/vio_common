# Estimate rolling shutter skew with LED panel

Most likely, the rolling shutter skew is independent of the exposure time.

## Data collection
adjust exposure time, and LED light time

## RS skew calculation
The principle to estimate the rolling shutter skew is that 
the elapsed time represented by number of lit LED columns is 
equal to that of the corresponding frame read out time.

### Assumptions
* The camera view is parallel to the LED panel and its x and y axes are aligned with those of the LED panel.

To solve for the rolling shutter skew, we need the following variables: 
m: slope caused by the rolling shutter effect
c: number of LED time steps, i.e., lit LED columns - 1
w: width of the lit LED columns in pixels
H: height of the image in pixels
t_led: amount of time a column of LEDs is lit for
t_r: frame readout time

t_r / H * m * w = c * t_led
thus,
t_r = c * t_led * H / (m * w)

### Method 1. Google ITS automatic detection and computation
python script calling opencv2 methods
The method is imprecise and does not work for long exposure time when 
overexposure prevent successful detection of the largest LED cluster.

### Method 2. Interactive drawing
Delineate slope line 1
Delineate slope line 2
Delineate left line through the centers of one column, say i at pixel pi in x
Delineate right line through the centers of another column, say j at pixel pj in x
The average slope of these two slope lines is m.
t_r = t_led * (j - i) * H / (m * (pj - pi))


