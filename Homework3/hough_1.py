An implementation of the Hough transform.
This program supports "Pixels, Numbers and Programs"
(chapter on Image Analysis)

1. read in an image.
    A demo image containing strong lines has been included:
    "floor.jpg"
    To avoid overly long compute times, use the 100 x 67 version.
    Otherwise, use the 200 x 134 version

2. compute edge strength.
    We use the Roberts cross operator.

3. take Hough transform.
    Note that the parameter space is based on a polar representation
    of lines where the pole is at the center of the image.  This means
    a translation (shift) by half the width and half the height when using
    PixelMath (x,y) coordinates in the line formula.
       Instead of the standard  rho = x cos theta + y sin theta
       we use  rho = (x - w/2) theta + (y - h/2) sin theta
4. suppress secondary maxima.
    Two simple schemes are provided: (a) scan the Hough array and whenever
    a value is max in its neighborhood, lower the neighbors by a factor of 0.5.
    This has a disadvantage in making lots of local maxes by virtue of one
    peak's neighborhood suppression makes it easier for the next non-neighbor
    to win, because his neighbors have been suppressed.
    (b) scan the Hough array and whenever a value is max in its neighborhood,
    flag, but do not lower the values of, the neighbors.  This is accomplished
    with a separate array called NONPEAK whose values are initially all False,
    and some of which get changed to True.  After this is done, a local max is
    a peak only if it is also not a nonpeak.
5. detect peaks.
    Works in conjunction with the previous step.
6. report dominant lines
    Indicates both the indexes (theta_idx and rho_idx) of the peak in the Hough
    array, and the corresponding values of theta and rho.
    Values of theta range from 0 to 2 pi (HN - 1)/HN,   and
    values of rho range from 0 to half_diag, which is the distance, in pixel
    widths, from the center of the original image to a corner of it.
7. create composite of original with detected lines.
    This is performed using a different version of the original, for aesthetic
    reasons.  Its resolution is double, and it is darkened by a factor of 0.5,
    so that the white lines plotted on it will show up well.

Characteristics:

flexible input resolution (automatically detects width and height)

Hough array dimensions specified as global variables.

Thresholds for secondary max suppression specified as globals.

Hough array represented as Python array (?)

'''

ORIG = pmOpenTwo("floor100x67-monochrome.jpg")
EDGES = ORIG+1
pmSetDestination(EDGES)
pmSetFormula("sqrt(sqr(s1(x,y)-s1(x-1,y-1))+sqr(s1(x-1,y)-s1(x,y-1)))")
pmCompute()

# Dimensions of Hough array:
HM = 128  # number of different rho values
HN = 128  # number of different theta values

HRow = HM*[0.0]  # an empty row of the Hough array.
H = [HRow[:] for i in range(HN)] # The whole, empty Hough array
EDGE_THRESH = 50 # Only pixels with edge value greater get to vote
HIT_THRESH = 1   # Pixels must lie within distance of 1 (pixel width)
                 # of a line for that line to receive the vote.
RAD = 3          # "radius" of neighborhood used in peak det. and supp.
                 # The neighborhoods, will be of size (2*RAD + 1)^2
                 # except at the borders of the image where they're smaller.
SUPRESSION_FACTOR = 0.5  # How much to reduce off-peak values.
HALF_DIAG = None # global that indicates range of rho values.

import math

def houghXform():
  global EDGES, HALF_DIAG
  w = pmGetImageWidth(EDGES)
  h = pmGetImageHeight(EDGES)
  diag = math.sqrt(w*w + h*h)
  HALF_DIAG = diag / 2.0
  for x_idx in range(1,w):
    print ".", # show some progress.
    x = x_idx - w/2.0
    for y_idx in range(1,h):
      y = y_idx - h/2.0
      (r,g,b) = pmGetPixel(EDGES,x_idx,y_idx)
      #monochr = (r+g+b)/3.0 # Use only if color orig.
      monochr = r
      if monochr > EDGE_THRESH:
        for rho_idx in range(HM):
          rho = HALF_DIAG * rho_idx/HM
          for theta_idx in range(HN):
            theta = math.pi * 2 * theta_idx / HN
            if abs(x * math.cos(theta) + y * math.sin(theta) - rho)\
               < HIT_THRESH:
              H[theta_idx][rho_idx] += monochr

def showHough():
  global H
  temp = pmNewImage(0, "Hough array", HM, HN, 0, 0, 0)
  pmPositionWindow(temp, 500, 500, 400, 400)
  for i in range(2): pmZoom(temp, 200, 200)
  for rho_idx in range(HM):
    for theta_idx in range(HN):
      r = g = b = int(H[theta_idx][rho_idx]/100)
      pmSetPixel(temp, theta_idx, rho_idx,  r, g, b)

HRow = HM*[False]
NOTPEAK = [HRow[:] for i in range(HN)]

def suppressSecondary():
  for rho_idx in range(HM):
    for theta_idx in range(HN):
      if isLocalMax(rho_idx, theta_idx):
        suppress_neighbors(rho_idx, theta_idx)

def isLocalMax(ri, ti):
  global NOTPEAK
  if NOTPEAK[ti][ri]: return False
  startRI = max(0, ri-RAD)
  endRI = min(HM, ri+RAD)
  startTI = max(0, ti-RAD)
  endTI = min(HN, ti+RAD)
  refValue = H[ti][ri]
  for rii in range(startRI, endRI):
    for tii in range(startTI, endTI):
      testValue = H[tii][rii]
      if testValue > refValue: return False
  return True

def suppress_neighbors(ri, ti):
  startRI = max(0, ri-RAD)
  endRI = min(HM, ri+RAD)
  startTI = max(0, ti-RAD)
  endTI = min(HN, ti+RAD)
  refValue = H[ti][ri]
  for rii in range(startRI, endRI):
    for tii in range(startTI, endTI):
      testValue = H[tii][rii]
      if testValue < refValue:
        NOTPEAK[tii][rii] = True

def findPeaks():
  global HALF_DIAG, NOTPEAK
  for rho_idx in range(HM):
    for theta_idx in range(HN):
      if NOTPEAK[theta_idx][rho_idx]: continue
      if H[theta_idx][rho_idx] > 7000:
        if isLocalMax(rho_idx, theta_idx):
          rho = (rho_idx * HALF_DIAG) / HM
          theta = theta_idx * 2 * math.pi / HN
          print "Peak of strength "+str(int(H[theta_idx][rho_idx]/100))+\
                " found at ("+str(theta_idx)+","+str(rho_idx)+");"+\
                " theta = "+str(theta)+"; rho = "+str(rho)+"."
          plotline(rho, theta)

PLOTWIN = None
BACKGDWIN = None
def plotline(rho, theta):
  global PLOTWIN, BACKGDWIN, HALF_DIAG
  if not PLOTWIN:
    BACKGDWIN = pmOpenImage(0, "floor200x134-monochrome.jpg")
    PLOTWIN   = pmOpenImage(0, "floor200x134-monochrome.jpg")
    HALF_DIAG = math.sqrt(pmGetImageWidth(PLOTWIN)**2 +\
                          pmGetImageHeight(PLOTWIN)**2)/2
    pmSetSource1(BACKGDWIN)
    pmSetDestination(PLOTWIN)
    pmSetFormula("S1(x,y)/2")
    pmCompute()
  pmSetSource1(BACKGDWIN)
  pmSetDestination(PLOTWIN)
  pmSetFormula("dest(x,y)+if abs((x-w/2) * cos("+str(theta)+")+ "+\
               "(y-h/2) * sin("+str(theta)+") - "+\
               str(rho)+") < 0.8 then 255 else 0")
  pmCompute()


houghXform()
showHough()
suppressSecondary()
findPeaks()
showHough()