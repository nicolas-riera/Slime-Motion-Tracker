"""Draws the SteamVR status icons of the virtual trackers (32x32 PNG, light glyph on a
transparent background) into driver/package/resources/icons. The driver references
them as "{slime_motion_tracker}/icons/tracker_<state>.png" (see virtual_tracker.cpp).

The glyph is our tracker: a rounded box on a strap, with the round cutout of the
SMT logo. States: ready (solid), off (dim outline), searching (faded fill, solid
outline), not ready (grey), standby (dim), and alert variants with an orange dot;
alert_low (low battery) has a red battery.

Usage: python tools/icon/make_tracker_icons.py [preview.png]
"""
import os
import sys

from PIL import Image, ImageDraw

SIZE = 32
SCALE = 8  # supersampling
OUT = os.path.join(os.path.dirname(__file__), "..", "..", "driver", "package", "resources", "icons")

WHITE = (235, 235, 240)
GREY = (150, 150, 158)
DIM = (105, 105, 112)
ORANGE = (245, 166, 35)
RED = (232, 72, 60)


def s(v):
    return round(v * SCALE)


def box(x0, y0, x1, y1):
    return [s(x0), s(y0), s(x1), s(y1)]


def tracker(color, fill_alpha=255, outline=False, line_alpha=255):
    """The tracker glyph, as an RGBA layer at the supersampled size."""
    layer = Image.new("RGBA", (s(SIZE), s(SIZE)), (0, 0, 0, 0))
    mask = Image.new("L", layer.size, 0)
    d = ImageDraw.Draw(mask)
    w = s(1.6)  # outline width
    # Strap behind the box
    strap = box(2, 13, 30, 19)
    body = box(7, 6, 25, 26)
    if outline:
        d.rounded_rectangle(strap, radius=s(1.5), fill=line_alpha)
        d.rounded_rectangle(box(2 + 1.6, 13 + 1.6, 30 - 1.6, 19 - 1.6), radius=s(0.5), fill=0)
    else:
        d.rounded_rectangle(strap, radius=s(1.5), fill=fill_alpha)
    # A gap between the strap and the box keeps them apart at 32 px
    d.rounded_rectangle(box(5.5, 4.5, 26.5, 27.5), radius=s(6), fill=0)
    d.rounded_rectangle(body, radius=s(5), fill=line_alpha if outline or fill_alpha < 255 else fill_alpha)
    if outline or fill_alpha < 255:
        d.rounded_rectangle([body[0] + w, body[1] + w, body[2] - w, body[3] - w], radius=s(5) - w,
                            fill=0 if outline else fill_alpha)
    # Logo cutout (outline and faded styles: a ring)
    cx, cy, r = s(16), s(16), s(4)
    if outline or fill_alpha < 255:
        d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=line_alpha)
        d.ellipse([cx - r + w, cy - r + w, cx + r - w, cy + r - w], fill=0)
    else:
        d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=0)
    layer.paste(Image.new("RGBA", layer.size, color + (255,)), (0, 0), mask)
    return layer


def badge(layer, color, battery=False):
    """Orange alert dot (or a low battery) in the top right corner, with a clear ring."""
    alpha = layer.getchannel("A")
    d = ImageDraw.Draw(alpha)
    if battery:
        clear, shape = box(15.5, 18.5, 32, 32), box(17.5, 21, 29.5, 29.5)
        d.rectangle(clear, fill=0)
    else:
        clear, shape = box(17.5, -0.5, 32.5, 14.5), box(20.5, 2.5, 29.5, 11.5)
        d.ellipse(clear, fill=0)
    layer.putalpha(alpha)
    d = ImageDraw.Draw(layer)
    if battery:
        d.rectangle(shape, outline=color + (255,), width=s(1.5))
        d.rectangle(box(29.5, 23.5, 31, 27), fill=color + (255,))  # terminal
        d.rectangle(box(19.5, 23, 22, 27.5), fill=color + (255,))  # last bar
    else:
        d.ellipse(shape, fill=color + (255,))
    return layer


def finish(layer):
    return layer.resize((SIZE, SIZE), Image.LANCZOS)


def icons():
    ready = lambda: tracker(WHITE)
    searching = lambda: tracker(WHITE, fill_alpha=90)
    standby = lambda: tracker(DIM)
    return {
        "tracker_ready": ready(),
        "tracker_ready_alert": badge(ready(), ORANGE),
        "tracker_off": tracker(DIM, outline=True),
        "tracker_searching": searching(),
        "tracker_searching_alert": badge(searching(), ORANGE),
        "tracker_not_ready": tracker(GREY),
        "tracker_standby": standby(),
        "tracker_standby_alert": badge(standby(), ORANGE),
        "tracker_alert_low": badge(ready(), RED, battery=True),
    }


def main():
    os.makedirs(OUT, exist_ok=True)
    images = {name: finish(img) for name, img in icons().items()}
    for name, img in images.items():
        img.save(os.path.join(OUT, name + ".png"), optimize=True)
    print(f"wrote {len(images)} icons to {os.path.normpath(OUT)}")
    if len(sys.argv) > 1:
        # Every icon at 4x on a dark and a light strip, for a quick look
        zoom = 4
        sheet = Image.new("RGBA", (len(images) * (SIZE + 4) * zoom, 2 * SIZE * zoom), (30, 30, 34, 255))
        ImageDraw.Draw(sheet).rectangle([0, SIZE * zoom, sheet.width, sheet.height], fill=(200, 200, 205, 255))
        for i, img in enumerate(images.values()):
            big = img.resize((SIZE * zoom, SIZE * zoom), Image.NEAREST)
            for row in range(2):
                sheet.alpha_composite(big, (i * (SIZE + 4) * zoom, row * SIZE * zoom))
        sheet.save(sys.argv[1])


if __name__ == "__main__":
    main()
