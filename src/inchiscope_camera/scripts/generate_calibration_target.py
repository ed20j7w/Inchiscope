#!/usr/bin/env python3
"""Generates a print-ready checkerboard calibration target as a PDF.

The NanEye/OV6948-class micro camera used here has a very short working
distance (endoscopic, focuses at a few mm to a couple of cm), so a normal
desk-calibration checkerboard (10-30mm squares) is far too big -- the board
would need to be held further away than the lens can focus, or wouldn't fit
in frame at all. This draws several square sizes on one page (vector PDF,
not a rasterised image) so printing at "Actual Size" / 100% scale -- NOT
"fit to page" -- gives exact physical dimensions. A ruled scale bar is
included so you can verify with an actual ruler that the printer didn't
silently rescale the page.

Usage:
    python3 generate_calibration_target.py [output.pdf]

Then: measure the scale bar with a ruler once printed. If it doesn't read
50mm, your print settings rescaled the page -- turn off any "fit to page"
/ "shrink to fit" option and reprint before using this for calibration.
"""

import sys

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# Standard OpenCV convention: internal corners, not squares. 9x6 internal
# corners (10x7 squares) is the classic default findChessboardCorners size
# and is asymmetric, so board orientation is never ambiguous.
SQUARES_X = 10
SQUARES_Y = 7

# Square sizes to print, in mm. Cut out whichever one actually fits inside
# your camera's field of view at its in-focus working distance -- see the
# sizing formula in the module docstring / README. These are reasonable
# starting guesses for a very-short-working-distance micro camera, not a
# confirmed number for this specific lens.
SQUARE_SIZES_MM = [1.0, 2.0, 3.0, 5.0, 8.0]

PAGE_WIDTH_MM = 210.0   # A4 portrait
PAGE_HEIGHT_MM = 297.0
MARGIN_MM = 10.0


def draw_checkerboard(ax, origin_x_mm, origin_y_mm, square_mm):
    for row in range(SQUARES_Y):
        for col in range(SQUARES_X):
            if (row + col) % 2 == 0:
                continue  # leave every other square white (paper background)
            ax.add_patch(Rectangle(
                (origin_x_mm + col * square_mm, origin_y_mm + row * square_mm),
                square_mm, square_mm,
                facecolor='black', edgecolor='none',
            ))
    width_mm = SQUARES_X * square_mm
    height_mm = SQUARES_Y * square_mm
    ax.add_patch(Rectangle(
        (origin_x_mm, origin_y_mm), width_mm, height_mm,
        facecolor='none', edgecolor='black', linewidth=0.3,
    ))
    return width_mm, height_mm


def draw_scale_bar(ax, origin_x_mm, origin_y_mm):
    """50mm bar with 10mm ticks -- measure this after printing to confirm
    the page wasn't rescaled."""
    ax.add_patch(Rectangle((origin_x_mm, origin_y_mm), 50.0, 3.0,
                            facecolor='black', edgecolor='none'))
    for i in range(6):
        x = origin_x_mm + i * 10.0
        ax.plot([x, x], [origin_y_mm - 1.5, origin_y_mm + 4.5], color='black', linewidth=0.5)
    ax.text(origin_x_mm, origin_y_mm + 6, 'Scale check: this bar must measure 50mm '
            'after printing. If not, disable "fit to page" and reprint.',
            fontsize=6)


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else 'calibration_target.pdf'

    fig_w_in = PAGE_WIDTH_MM / 25.4
    fig_h_in = PAGE_HEIGHT_MM / 25.4
    fig, ax = plt.subplots(figsize=(fig_w_in, fig_h_in))
    ax.set_xlim(0, PAGE_WIDTH_MM)
    ax.set_ylim(0, PAGE_HEIGHT_MM)
    ax.set_aspect('equal')
    ax.axis('off')
    fig.subplots_adjust(left=0, right=1, top=1, bottom=0)

    y_cursor = PAGE_HEIGHT_MM - MARGIN_MM
    for square_mm in SQUARE_SIZES_MM:
        width_mm, height_mm = draw_checkerboard(ax, MARGIN_MM, y_cursor - SQUARES_Y * square_mm, square_mm)
        ax.text(MARGIN_MM, y_cursor + 3,
                f'{square_mm:g}mm squares -- {SQUARES_X}x{SQUARES_Y} squares, '
                f'{SQUARES_X - 1}x{SQUARES_Y - 1} internal corners -- '
                f'board size {width_mm:g}x{height_mm:g}mm',
                fontsize=7)
        y_cursor -= (SQUARES_Y * square_mm + 15.0)

    draw_scale_bar(ax, MARGIN_MM, MARGIN_MM)

    fig.savefig(out_path, format='pdf')
    print(f'Wrote {out_path}')
    print('Print at "Actual Size" / 100% scale, NOT "fit to page".')
    print('Verify the scale bar measures exactly 50mm before using this for calibration.')


if __name__ == '__main__':
    main()
