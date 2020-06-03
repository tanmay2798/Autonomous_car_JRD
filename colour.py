from PIL import Image
import numpy as np

im = Image.open('Marketing.png')
im = im.convert('RGBA')

data = np.array(im)   # "data" is a height x width x 4 numpy array
red, green, blue, alpha = data.T # Temporarily unpack the bands for readability

# Replace white with red... (leaves alpha values alone...)
for r in range(230,250):
	for g in range(230,250):
		for b in range(230,250):
			white_areas = (red == r) & (blue == b) & (green == g)
			data[..., :-1][white_areas.T] = (255, 255, 255) # Transpose back needed

im2 = Image.fromarray(data)
im2.show()