#!/usr/bin/env python3

from PIL import Image
import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('input', help='input image path')
    parser.add_argument('output', help='output image path')
    args = parser.parse_args()

    img = Image.open(args.input)
    img = img.rotate(90)

    if img.width != img.height:
        raise ValueError("Input image must be square")

    # increas the resolution of image by 10 times because the original image is too small!
    img = img.resize((img.width * 10, img.width * 10), Image.NEAREST)
    
    w = img.width
    w_B = int(w * (63 / 50))
    h_B = int(w * (55 / 50))
    margin_left = (w_B - w) // 2
    margin_top = (h_B - w) // 2
    
    new_img = Image.new('RGB', (w_B, h_B), (255, 255, 255))
    new_img.paste(img, (margin_left, margin_top))
    new_img.save(args.output)

if __name__ == '__main__':
    main()
