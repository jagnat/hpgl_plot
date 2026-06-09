"""svg2hpgl.py - Convert an SVG into HP-GL records for an HP 7440A ColorPro pen plotter.

The companion to svg2gcode.py. Instead of emitting G-code for the 3D-printer
plotter, this emits HP-GL (Hewlett-Packard Graphics Language), the language the
HP 7440A speaks over RS-232.

The output is written as a *record file*: one HP-GL instruction per line, each
line guaranteed to be <= MAX_RECORD bytes. This matters because the base 7440A
(no Graphics Enhancement Cartridge) has a tiny input buffer and no robust flow
control. The companion streamer, hpgl_stream.py, sends these records one at a
time using the plotter's Enquire/Acknowledge handshake so the buffer is never
overrun.

Pipeline (mirrors svg2gcode.py):
    SVG line/circle/ellipse elements --> SVG <path>  (convert_to_path)
    parse_root --> flattened line-segment curves     (svg_to_gcode)
    curves --> polylines in plotter units            (this file)
    polylines --> chunked PU/PD records              (this file)

HP-GL primer (what we actually emit):
    IN;            initialize the plotter
    SP1;           select pen 1
    PU x,y;        pen up, move to (x,y)        -- travel, no line
    PD x,y,...;    pen down, draw through points -- a PD takes a coord list
    PU;            pen up (park)
    SP0;           return pen to the carousel
Coordinates are integer *plotter units*: 40 units/mm (1016 units/inch). The
plotter origin is bottom-left and Y grows upward, so we flip Y from SVG space.
"""

import os
import re
import sys
import math
import datetime
from xml.etree import ElementTree

from svg_to_gcode.svg_parser import parse_root
from svg_to_gcode.geometry import LineSegmentChain, Line

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

# Input SVG. Override on the command line: python3 svg2hpgl.py path/to/file.svg
path = 'output_260608_174939.svg'

# Pen to draw with (1-8 on an 8-pen carousel; the 7440A holds up to 8).
pen = 6

# Plotter-unit drawing area to fit the artwork into. These are the HP 7440A
# US/Letter hard-clip limits (~10.1in x 7.5in landscape); 40 plotter units == 1mm.
# Verified on hardware: a box at (40,40)-(10260,7610) draws cleanly within reach.
plot_x_min = 0.0
plot_x_max = 10300.0
plot_y_min = 0.0
plot_y_max = 7650.0

# Extra margin (plotter units) kept clear inside the area above. Keeps the pen
# off the hard stops (40 units == 1mm).
margin = 40.0

# Rotate the artwork counter-clockwise by this many degrees before fitting.
# Use 90 to align a portrait drawing with landscape paper for a larger plot.
# The fit/center/flip step runs after rotation, so any pivot works.
rotate = 0

# Maximum bytes per emitted record (including the trailing ';'). The base 7440A
# accepts only small buffers, so keep this comfortably under 60. The streamer's
# Enquire/Acknowledge block size must be >= this value.
MAX_RECORD = 58

# Treat two points closer than this (plotter units) as coincident when deciding
# whether consecutive curves form one continuous polyline.
JOIN_EPSILON = 1e-3

# ---------------------------------------------------------------------------
# SVG element normalization (lifted from svg2gcode.py)
# ---------------------------------------------------------------------------

def convert_to_path(node, ns, style, d):
    node.clear()
    # parse_root only recognizes the namespaced {svg}path tag.
    node.tag = ns + 'path'
    if style is not None:
        node.set('style', style)
    node.set('d', d)


def _points_to_path(points, close):
    """Turn an SVG points="x,y x,y ..." list into a path 'd' string.

    Coordinates may be separated by commas and/or whitespace; we just pull out
    every number in order and pair them up. `close` adds a trailing 'Z' (polygon).
    """
    nums = [float(n) for n in re.findall(r'[-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?', points)]
    pts = list(zip(nums[0::2], nums[1::2]))
    if len(pts) < 2:
        return None
    d = 'M {} {}'.format(*pts[0])
    d += ''.join(' L {} {}'.format(x, y) for (x, y) in pts[1:])
    if close:
        d += ' Z'
    return d


def normalize_primitives(root, ns):
    """Rewrite primitive shapes as <path> so parse_root flattens them.

    Group <transform> attributes (translate/rotate/...) live on ancestor <g>
    elements, not on these shape nodes, so clearing/retagging a shape leaves the
    inherited transforms intact -- parse_root still applies them to the path.
    """
    for node in root.iter():
        if node.tag in (ns + 'polygon', ns + 'polyline'):
            points = node.get('points')
            if not points:
                continue
            d = _points_to_path(points, close=node.tag == ns + 'polygon')
            if d is None:
                continue
            convert_to_path(node, ns, node.get('style'), d)
        elif node.tag == ns + 'line':
            x1 = float(node.get('x1')); x2 = float(node.get('x2'))
            y1 = float(node.get('y1')); y2 = float(node.get('y2'))
            style = node.get('style')
            d = 'M {} {} L {} {}'.format(x1, y1, x2, y2)
            convert_to_path(node, ns, style, d)
        elif node.tag == ns + 'rect':
            x = float(node.get('x', 0)); y = float(node.get('y', 0))
            w = float(node.get('width', 0)); h = float(node.get('height', 0))
            if w <= 0 or h <= 0:
                continue
            style = node.get('style')
            # Sharp-cornered rectangle as a closed path. (rx/ry rounding is
            # ignored -- these SVGs use plain rects.)
            d = 'M {0} {1} L {2} {1} L {2} {3} L {0} {3} Z'.format(x, y, x + w, y + h)
            convert_to_path(node, ns, style, d)
        elif node.tag == ns + 'circle':
            r = float(node.get('r'))
            cx = float(node.get('cx')); cy = float(node.get('cy'))
            style = node.get('style')
            d = 'M {1} {0} A {3} {3} 0 0 0 {2} {0} A {3} {3} 0 0 0 {1} {0}' \
                .format(cy, cx - r, cx + r, r)
            convert_to_path(node, ns, style, d)
        elif node.tag == ns + 'ellipse':
            cx = float(node.get('cx')); cy = float(node.get('cy'))
            rx = float(node.get('rx')); ry = float(node.get('ry'))
            style = node.get('style')
            d = 'M {1} {0} A {3} {4} 0 0 0 {2} {0} A {3} {4} 0 0 0 {1} {0}' \
                .format(cy, cx - rx, cx + rx, rx, ry)
            convert_to_path(node, ns, style, d)


# ---------------------------------------------------------------------------
# Curves --> polylines
# ---------------------------------------------------------------------------

def curves_to_polylines(curves):
    """Flatten svg_to_gcode curves into a list of polylines (lists of (x, y)).

    Each curve is approximated by straight segments; consecutive curves whose
    endpoints touch are merged into one continuous polyline (one pen-down run).
    """
    polylines = []
    current = None

    for curve in curves:
        chain = LineSegmentChain.line_segment_approximation(curve)
        segments = list(chain)
        if not segments:
            continue

        start = segments[0].start
        if current is None or _dist(current[-1], (start.x, start.y)) > JOIN_EPSILON:
            current = [(start.x, start.y)]
            polylines.append(current)

        for seg in segments:
            current.append((seg.end.x, seg.end.y))

    return polylines


def _dist(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def rotate_polylines(polylines, deg):
    """Rotate every point CCW by `deg` about the origin. build_transform re-fits
    afterward, so the pivot is irrelevant -- only the orientation matters."""
    if deg % 360 == 0:
        return polylines
    rad = math.radians(deg)
    c, s = math.cos(rad), math.sin(rad)
    return [[(x * c - y * s, x * s + y * c) for (x, y) in poly] for poly in polylines]


# ---------------------------------------------------------------------------
# SVG space --> plotter units
# ---------------------------------------------------------------------------

def build_transform(polylines):
    """Return f(x, y) -> (ix, iy) mapping SVG coords into integer plotter units,
    scaled to fit the configured area (aspect preserved) with Y flipped."""
    xs = [p[0] for poly in polylines for p in poly]
    ys = [p[1] for poly in polylines for p in poly]
    if not xs:
        raise ValueError('No drawable geometry found in SVG.')

    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    src_w = max(max_x - min_x, 1e-9)
    src_h = max(max_y - min_y, 1e-9)

    avail_w = (plot_x_max - plot_x_min) - 2 * margin
    avail_h = (plot_y_max - plot_y_min) - 2 * margin
    scale = min(avail_w / src_w, avail_h / src_h)

    # Center the artwork within the available area.
    off_x = plot_x_min + margin + (avail_w - src_w * scale) / 2.0
    off_y = plot_y_min + margin + (avail_h - src_h * scale) / 2.0

    def transform(x, y):
        ix = off_x + (x - min_x) * scale
        # Flip Y: SVG origin is top-left, plotter origin is bottom-left.
        iy = off_y + (max_y - y) * scale
        return int(round(ix)), int(round(iy))

    return transform


# ---------------------------------------------------------------------------
# HP-GL record emission
# ---------------------------------------------------------------------------

def emit_records(polylines, transform):
    """Yield HP-GL instruction strings, each <= MAX_RECORD bytes (incl. ';')."""
    yield 'IN;'
    yield 'SP{};'.format(pen)

    for poly in polylines:
        pts = [transform(x, y) for (x, y) in poly]
        # Drop consecutive duplicate plotter-unit points (rounding collisions).
        deduped = [pts[0]]
        for pt in pts[1:]:
            if pt != deduped[-1]:
                deduped.append(pt)

        x0, y0 = deduped[0]
        yield 'PU{},{};'.format(x0, y0)

        if len(deduped) < 2:
            # Degenerate polyline -- a single point at plotter resolution (the
            # source is a zero-length "dot" line, or a tiny shape that rounded to
            # one cell). Set the pen down on the spot so the dot isn't lost.
            yield 'PD{},{};'.format(x0, y0)
            continue

        # Pack the remaining points into one or more PD records. The pen stays
        # down across consecutive PD instructions, so a long polyline can be
        # split freely without lifting; each split just continues from the
        # current pen position.
        rec = 'PD'
        for (x, y) in deduped[1:]:
            token = '{},{}'.format(x, y)
            sep = '' if rec == 'PD' else ','
            # +1 for the trailing ';'
            if len(rec) + len(sep) + len(token) + 1 > MAX_RECORD:
                yield rec + ';'
                rec = 'PD' + token
            else:
                rec += sep + token
        if rec != 'PD':
            yield rec + ';'

    yield 'PU;'
    yield 'SP0;'


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    in_path = sys.argv[1] if len(sys.argv) > 1 else path

    ElementTree.register_namespace('', 'http://www.w3.org/2000/svg')
    root = ElementTree.parse(in_path).getroot()
    ns = '{http://www.w3.org/2000/svg}'

    normalize_primitives(root, ns)

    curves = parse_root(root, transform_origin=False)
    polylines = curves_to_polylines(curves)
    polylines = rotate_polylines(polylines, rotate)
    transform = build_transform(polylines)
    records = list(emit_records(polylines, transform))

    # Sanity: no record may exceed the buffer budget.
    too_long = [r for r in records if len(r) > MAX_RECORD]
    if too_long:
        raise AssertionError('Records exceed MAX_RECORD: {!r}'.format(too_long[:3]))

    os.makedirs('hpgl', exist_ok=True)
    out = 'hpgl/gen_' + datetime.datetime.now().strftime('%y%m%d_%H%M%S') + '.hgl'
    with open(out, 'w') as f:
        f.write('\n'.join(records) + '\n')

    draw_records = sum(1 for r in records if r.startswith('PD'))
    print('Wrote {} ({} records, {} PD, {} polylines).'.format(
        out, len(records), draw_records, len(polylines)))
    print('Stream it with:  python3 hpgl_stream.py {} <serial-port>'.format(out))


if __name__ == '__main__':
    main()
