""" Module that contains functions for drawing the slagsquare groundplan
"""

import cv2
import numpy as np
import time
from datetime import datetime
import pytz


HOT_TEMPERATURE_THRESHOLD = 300
WARM_TEMPERATURE_THRESHOLD = 200
R_HOT_TEMPERATURE_THRESHOLD = 300
R_WARM_TEMPERATURE_THRESHOLD = 200
COLORS = {
    'black': (0, 0, 0),
    'white': (255, 255, 255),
    'grey': (100, 100, 100),
    'light_grey': (150, 150, 150),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'yellow': (0, 255, 255),
    'red': (0, 0, 255)
}


def draw_groundplan(shape, grid_shape, states, stamp,
                    git_version='', git_hash='', warning=None):
    """ Draws the slagsquare groundplan

        Keyword arguments:
        scale -- The width and height of the image to generate in pixels
        grid -- The number of rows and cols of pots in the slagsquare
        states -- The pot states to visualize
    """
    # create blank canvas
    groundplan = (255 * np.ones(shape)).astype(np.uint8)

    h_step = 32
    v_step = 65
    origin = (40, 15)

    # draw legend of pot states
    draw_legend(
        img=groundplan,
        origin=origin,
        radius=8)

    # draw 2d grid
    draw_grid(
        img=groundplan,
        origin=(origin[0]-10, origin[1] + 40),
        dims=grid_shape,
        step=(h_step,v_step),
        color=COLORS['light_grey'],
        thickness=1)

    # draw horizontal and vertical grid numbering
    draw_grid_numbering(
        img=groundplan,
        h_nums=range(1,29),
        v_nums='ABCDEFGH',
        h_offset=(origin[0]-3, origin[1]+30),
        v_offset=(origin[0]-30, origin[1] + v_step + 10),
        h_step=h_step,
        v_step=v_step)

    # draw the R labels on the top of the 26th and 27th columns
    draw_R_pot_labels(
        img=groundplan,
        nums=range(1, 3),
        offset=(origin[0] + h_step * 24, 25),
        step=30)

    draw_keizer_pot_labels(
        img=groundplan,
        nums=range(1, 11),
        offset=(origin[0] + h_step * 8,
                origin[1] + v_step * grid_shape[0] + v_step - 5),
        step=int(h_step * 1.5))

    # draw timestamp of visualization and of last state update
    draw_timestamp(
        img=groundplan,
        stamp=stamp,
        origin=(shape[1] - 250, shape[0] - 25))

    # draw version of application
    draw_version(
        img=groundplan,
        origin=(10, shape[0] - 10),
        git_version=git_version,
        git_hash=git_hash)

    # draw pot states
    draw_pot_states(
        img=groundplan,
        states=states,
        origin=(origin[0]-5, origin[1]+3*v_step//4),
        step=(h_step, v_step),
        radius=10)

    # if latest update is older than 5 minutes display indication of failure
    warning_org = (500, 25)
    if (time.time() - stamp) > 5 * 60:
        draw_warning(
            img=groundplan,
            text="SYSTEM FAILURE",
            origin=warning_org)
        #  draw_failure_indication(img=groundplan) # overlay put in streamer
    else: # draw warning about application being under test
        draw_warning(
            img=groundplan,
            text=warning,
            origin=warning_org)

    return groundplan


def draw_grid(img, origin, dims, step=[50,50], color=COLORS['black'],
              thickness=1, line_type=cv2.LINE_AA):
    """ Draws a grid representing the slagsquare

        Keyword arguments:
        img -- The img to draw the grid on
        origin -- The upper left corner position of the grid
        dims -- The number of rows and cols of the grid
        color -- The color of the grid lines (default: black)
        thickness -- The thickness of the grid lines (default: 1)
        step -- The step in pixels between grid lines
    """
    x0,y0 = origin

    # calculate vertical and horizontal limits of grid
    x_limit = x0 + dims[1] * step[0]
    xk0 = x0 + step[0] * 8
    xk_limit = xk0 + 500
    y_limit = y0 + dims[0] * step[1]

    # draw vertical lines
    for idx, x in enumerate(range(x0, x_limit+1, step[0])):
        ylim = y_limit

        # skip keizer pot lines
        if 8 < idx < 23:
            ylim -= step[1]

        cv2.line(img,
                 (x, y0),
                 (x, ylim),
                 color=color,
                 lineType=cv2.LINE_AA,
                 thickness=thickness)

    # draw keizer pot vertical lines
    for idx, x in enumerate(range(xk0, xk_limit, int(step[0]*1.5))):
        cv2.line(img,
                 (x, y_limit-step[1]),
                 (x, y_limit),
                 color=color,
                 lineType=cv2.LINE_AA,
                 thickness=thickness)

    # draw horizontal lines
    for y in range(y0, y_limit+1, step[1]):
        cv2.line(img,
                 (x0, y),
                 (x_limit, y),
                 color=color,
                 lineType=cv2.LINE_AA,
                 thickness=thickness)

    #  draw thick line splitting the grid in elevated and lower cooling fields
    cv2.line(img,
             (x0 + 8 * step[0], y0),
             (x0 + 8 * step[0], ylim),
             color=color,
             lineType=cv2.LINE_AA,
             thickness=3)


def draw_grid_numbering(img, h_nums, v_nums, h_offset, v_offset,
                        h_step, v_step):
    """ Draws the horizontal and vertical scale numbering

        Keyword arguments:
        img -- The image to draw the numbering on
        h_nums -- The horizontal numbering
        v_nums -- The vertical numbering
        h_offset -- The offset in (x,y) of the horizontal numbering
        v_offset -- The offset in (x,y) of the vertical numbering
        h_step -- The step in pixels of the horizontal numbering
        v_step -- The step in pixels of the vertical numbering
    """
    # draw horizontal numbering
    for idx, h_num in enumerate(h_nums):
        cv2.putText(
            img=img,
            text='{:02d}'.format(h_num),
            org=(h_offset[0] + idx * h_step, h_offset[1]),
            fontFace=cv2.FONT_HERSHEY_DUPLEX,
            fontScale=0.5,
            color=COLORS['black'],
            thickness=1,
            lineType=cv2.LINE_AA)

    # draw vertical numbering
    for idx, v_num in enumerate(v_nums):
        cv2.putText(
            img=img,
            text=v_num,
            org=(v_offset[0], v_offset[1] + idx * v_step),
            fontFace=cv2.FONT_HERSHEY_DUPLEX,
            fontScale=0.5,
            color=COLORS['black'],
            thickness=1,
            lineType=cv2.LINE_AA)


def draw_R_pot_labels(img, nums, offset, step):
    """ Draws the R labels on the given columns

        Keyword arguments:
        img -- The input image to annotate
        nums -- The indices of the R columns
        offset -- The location of the first label
        step -- The horizontal offset between two labels
    """
    # draw R1 and R2 numberings
    for idx in nums:
        cv2.putText(
            img=img,
            text='R{}'.format(idx),
            org=(offset[0] + idx * step, offset[1]),
            fontFace=cv2.FONT_HERSHEY_DUPLEX,
            fontScale=0.6,
            color=COLORS['blue'],
            thickness=1,
            lineType=cv2.LINE_AA)


def draw_keizer_pot_labels(img, nums, offset, step):
    """ Draws the Keizer pot labels

        Keyword arguments:
        img -- The input image to annotate
        nums -- The indices of the Keizer pots
        offset -- The location of the first label
        step -- The horizontal offset between two labels
    """
    for idx, num in enumerate(nums):
        cv2.putText(
            img=img,
            text='K{}'.format(num),
            org=(offset[0] + idx * step, offset[1]),
            fontFace=cv2.FONT_HERSHEY_DUPLEX,
            fontScale=0.6,
            color=COLORS['blue'],
            thickness=1,
            lineType=cv2.LINE_AA)


def draw_legend(img, origin, radius):
    """ Draws the legend explaining what the different pot symbols mean

        Keyword arguments:
        img -- The image to draw on
        origin -- The position where to put the legend
        radius -- The radius of the pot symbols
    """
    x0, y0 = origin
    dx, dy = radius*2, radius // 2

    # draw cold pot legend
    draw_legend_pot(
        img=img,
        origin=(x0,y0),
        delta=(dx,dy),
        radius=radius,
        color=COLORS['green'],
        str_top='Cold',
        str_bottom=f'T<{WARM_TEMPERATURE_THRESHOLD}C')

    # draw warm pot legend
    draw_legend_pot(
        img=img,
        origin=(x0 + 6 * dx, y0),
        delta=(dx, dy),
        radius=radius,
        color=COLORS['yellow'],
        str_top='Cooling',
        str_bottom=f'T<{HOT_TEMPERATURE_THRESHOLD}C')

    # draw hot pot legend
    draw_legend_pot(
        img=img,
        origin=(x0 + 12 * dx, y0),
        delta=(dx, dy),
        radius=radius,
        color=COLORS['red'],
        str_top='Hot',
        str_bottom=f'T>={HOT_TEMPERATURE_THRESHOLD}C')


def draw_legend_pot(img, str_top, str_bottom,
                    origin, delta=(10,10), radius=5, color=COLORS['green']):
    """ Draws a pot with text explaining its meaning

        Keyword arguments:
        img -- The image to draw on
        origin -- The position where to draw
        delta -- The distance in pixels between symbol and text
        radius -- The readius of the pot symbol
        color -- The color of the pot symbol
        str_top -- The string to put on the top beside the pot symbol
        str_bottom -- The string to put on the bottom beside the pot symbol
    """
    x0, y0 = origin
    dx, dy = delta

    # draw pot symbol
    cv2.circle(img, (x0,y0), radius, color, -1, cv2.LINE_AA)
    cv2.circle(img, (x0,y0), radius, COLORS['black'], 3, cv2.LINE_AA)

    # draw upper text beside pot symbol
    cv2.putText(
        img=img,
        text=str_top,
        org=(x0 + dx, y0 - dy // 2),
        fontFace=cv2.FONT_HERSHEY_DUPLEX,
        fontScale=0.4,
        color=COLORS['grey'],
        thickness=1,
        lineType=cv2.LINE_AA)

    # draw lower text beside pot symbol
    cv2.putText(
        img=img,
        text=str_bottom,
        org=(x0 + dx, y0 + 3 * dy),
        fontFace=cv2.FONT_HERSHEY_DUPLEX,
        fontScale=0.4,
        color=COLORS['grey'],
        thickness=1,
        lineType=cv2.LINE_AA)


def draw_timestamp(img, stamp, origin,
                   color1=COLORS['black'], color2=COLORS['red'],
                   font_scale = 0.4):
    """ Draw the timestamp of the visualization and of the latest update

        Keyword arguments:
        img -- The image to draw on
        stamp -- The latest update to timestamp
        origin -- The position to draw on
        color1 -- The color of the visualization timestamp
        color2 -- The color of the latest update timestamp
    """
    x0, y0 = origin
    h, _, d = img.shape
    w = 30

    # get current time
    current_time = datetime.now()

    # get timezone offset from UTC
    utcoffset = pytz.timezone('CET').utcoffset(current_time, is_dst = True)
    h_offset = int(utcoffset.total_seconds() / 3600)
    sign = '+' * ((h_offset) >= 0) + '-' * ((h_offset) < 0)
    if abs(h_offset) > 0:
        tz_suffix = ' UTC' + sign + str(h_offset)
    else:
        tz_suffix = ''

    # draw current time on canvas
    cv2.putText(
        img=img,
        text='Time: ' + current_time.strftime("%Y-%m-%d %H:%M:%S") + tz_suffix,
        org=(x0, y0),
        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
        fontScale=font_scale,
        color=color1,
        thickness=1,
        lineType=cv2.LINE_AA)

    if stamp is None:
        return

    # convert timestamp to datetime format
    frame_stamp = datetime.fromtimestamp(stamp)

    # draw timestamp on blank canvas
    cv2.putText(
        img=img,
        text='Last update: ' + \
            frame_stamp.strftime("%Y-%m-%d %H:%M:%S") + tz_suffix,
        org=(x0 - 48, y0 + 15),
        fontFace=cv2.FONT_HERSHEY_DUPLEX,
        fontScale=font_scale,
        color=color2,
        thickness=1,
        lineType=cv2.LINE_AA)


def draw_warning(img, text, origin, fontScale=0.8, thickness=1,
                 fontFace=cv2.FONT_HERSHEY_DUPLEX):
    """ Draw a warning that application is under test

        Keyword arguments:
        img -- The image to draw on
        origin -- The position to draw on
    """
    if text is None or text == '':
        return

    # calculate text size
    ret, baseline = cv2.getTextSize(
        text=text,
        fontFace=fontFace,
        fontScale=fontScale,
        thickness=thickness)
    dx, dy = ret

    # draw yellow background
    x0, y0 = origin
    offset = 6
    img[y0-dy-offset:y0+offset, x0-offset:x0+dx+offset] = COLORS['yellow']

    # draw text
    cv2.putText(
        img=img,
        text=text,
        org=origin,
        fontFace=fontFace,
        fontScale=fontScale,
        color=COLORS['black'],
        thickness=thickness,
        lineType=cv2.LINE_AA)

def draw_version(img, origin, git_version, git_hash,
                 fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.4,
                 color=COLORS['black'], thickness=1):
    """ Draw the git version of the code

        Keyword arguments:
        img -- The image to draw on
        origin -- The position to draw on
    """
    x0, y0 = origin

    # draw git version
    cv2.putText(
        img=img,
        text='Version: Alpha '+git_version,
        org=(x0, y0),
        fontFace=fontFace,
        fontScale=fontScale,
        color=color,
        thickness=thickness,
        lineType=cv2.LINE_AA)

    # draw commit hash
    cv2.putText(
        img=img,
        text=git_hash,
        org=(x0+300, y0),
        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
        fontScale=fontScale,
        color=color,
        thickness=thickness,
        lineType=cv2.LINE_AA)


def draw_pot_states(img, states, origin, step, radius):
    """ Draws the pot states

        Keyword arguments:
        img -- The image to draw on
        states -- The pot states
        origin -- The position to start drawing on
        step -- The step between two pot drawings
        radius -- The radius of a pot symbol
    """
    x0, y0 = origin
    dx, dy = step
    dxt, dyt = -12, 30

    # iterate through all pot states
    for xy in states:
        if xy not in states or 'temperature' not in states[xy]:
            #  print(f'Received invalid state with x,y={xy}')
            continue

        # calculate pot center
        if xy[1] < 7 or (xy[1] == 7 and (xy[0] >= 23 or 0 <= xy[0] < 8)):
            x = x0 + dx * xy[0] + 12
            y = y0 + dy * xy[1] + 15
        elif xy[1] == 7:
            if 8 <= xy[0] <= 17:  # keizer pot
                x = x0 + 8 * dx + int(dx * 1.5) * (xy[0]-8) + 18
                y = y0 + dy * xy[1] + 15
            elif 18 <= xy[0] <= 22:  # invalid pot
                print(f'Pot {xy} doesn\'t exist')
                continue
        else:
            # print(f'Pot slot [{xy[0]}, {xy[1]}] is invalid')
            continue

        # get color based on temperature value
        is_R_pot = xy[0] in range(26, 28)
        color = get_temperature_color(states[xy]['temperature'], is_R_pot)

        # draw side spouts in case of keizer pot
        if 8 <= xy[0] <= 17 and xy[1] == 7:
            cv2.line(img,
                     (x-15, y),
                     (x+15, y),
                     color=COLORS['black'],
                     lineType=cv2.LINE_AA,
                     thickness=9)

        # draw inner circle of pot symbol
        cv2.circle(
            img=img,
            center=(x, y),
            radius=radius,
            color=color,
            thickness=-1,
            lineType=cv2.LINE_AA)

        # draw outline of pot symbol
        cv2.circle(
            img=img,
            center=(x, y),
            radius=radius,
            color=COLORS['black'],
            thickness=3,
            lineType=cv2.LINE_AA)

        # draw temperature of pot
        cv2.putText(
            img=img,
            text='{:>3}'.format(int(states[xy]['temperature'])),
            org=(x + dxt, y + dyt),
            fontFace=cv2.FONT_HERSHEY_DUPLEX,
            fontScale=0.4,
            color=COLORS['black'],
            thickness=1,
            lineType=cv2.LINE_AA)


def draw_failure_indication(img):
    """ Overlays the image with red plus a warning message

        Keyword arguments:
        img -- The input image to overlay with the red mask
    """
    # create overlay failure image
    failure_overlay = np.zeros_like(img)
    failure_overlay[:, :, 2] = 255

    # blend image with overlay
    cv2.addWeighted(img, 0.5, failure_overlay, 0.5, 1, img)


def get_temperature_color(t, ir_R_pot=False):
    """ Returns color given temperature based on the following rule
        green (cold): below WARM_TEMPERATURE_THRESHOLD
        yellow (warm): between WARM_ and HOT_TEMPERATURE_THRESHOLD
        red (hot): above HOT_TEMPERATURE_THRESHOLD

        Keyword arguments:
        t -- The temperature
    """
    if ir_R_pot:
        t_h = R_HOT_TEMPERATURE_THRESHOLD
        t_l = R_WARM_TEMPERATURE_THRESHOLD
    else:
        t_h = HOT_TEMPERATURE_THRESHOLD
        t_l = WARM_TEMPERATURE_THRESHOLD


    if t >= t_h:
        color = COLORS['red']
    elif t >= t_l:
        color = COLORS['yellow']
    else:
        color = COLORS['green']

    return color
