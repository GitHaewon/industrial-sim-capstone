"""Generate the occupancy image paired with factory_map.yaml."""

from pathlib import Path


WIDTH = 240
HEIGHT = 180


def rectangle(pixels, x0, y0, x1, y1):
    """Mark an occupied rectangle in the map image."""
    for y in range(max(0, y0), min(HEIGHT, y1)):
        for x in range(max(0, x0), min(WIDTH, x1)):
            pixels[y * WIDTH + x] = 0


def main():
    pixels = bytearray([255] * (WIDTH * HEIGHT))
    rectangle(pixels, 8, 5, 232, 8)
    rectangle(pixels, 8, 172, 42, 176)
    rectangle(pixels, 198, 172, 232, 176)
    rectangle(pixels, 8, 5, 11, 176)
    rectangle(pixels, 229, 5, 232, 176)
    rectangle(pixels, 82, 83, 158, 97)
    rectangle(pixels, 127, 100, 138, 111)
    rectangle(pixels, 22, 20, 198, 30)
    rectangle(pixels, 22, 40, 198, 50)

    output = Path(__file__).with_name('factory_map.pgm')
    output.write_bytes(
        f'P5\n{WIDTH} {HEIGHT}\n255\n'.encode() + bytes(pixels)
    )


if __name__ == '__main__':
    main()
