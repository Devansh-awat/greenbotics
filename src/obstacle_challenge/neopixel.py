import board
import neopixel_spi

# --- Configuration ---
NUM_PIXELS = 64
# Calculated to keep current draw under 1.2A (1.2A / 3.84A max = 0.3125)
BRIGHTNESS = 0
PIXEL_ORDER = "GRB" # Common pixel order. Change to "RGB" if colors are off.

# Define the color white
WHITE = (255, 255, 255)

def init():
    global pixels
    spi = board.SPI()
    pixels = neopixel_spi.NeoPixel_SPI(
        spi,
        NUM_PIXELS,
        pixel_order=PIXEL_ORDER,
        brightness=BRIGHTNESS,
        auto_write=False,
    )
def solid(r,g,b):
        for x in range(32,64):
            pixels[x]=(r,g,b)
        pixels.show()
def cleanup():
    pixels.deinit()

if __name__ == "__main__":
    init()
    solid(255,255,255)
    try:
        while True:
            pass
    except KeyboardInterrupt:
        pass
    finally:
        cleanup()