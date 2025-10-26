from xml.dom import minidom
from svgpathtools import parse_path

def load_svg_paths(svg_file):
    doc = minidom.parse(svg_file)
    paths = []
    for path in doc.getElementsByTagName('path'):
        d = path.getAttribute('d')
        svg_path = parse_path(d)
        points = [(seg.start.real, seg.start.imag) for seg in svg_path]
        paths.append(points)
    return paths

print(load_svg_paths("butterfly.svg"))