#!/usr/bin/env python3

import os, sys, json, random, imutils, glob, random
import cv2
import numpy as np
import xml.etree.cElementTree as ET
from xml.dom import minidom

BGIMGDIR = 'bgimages'
OUTDIR = 'source'
IMGDIR = 'images'
LABELDIR = 'labels'
LABELPREVIEWDIR = 'label_previews'
XML_LOCATION = 'xml'

for dname in (os.path.join(OUTDIR, IMGDIR),os.path.join(OUTDIR, LABELDIR), os.path.join(OUTDIR, LABELPREVIEWDIR), os.path.join(OUTDIR, XML_LOCATION)):
	if not os.path.exists(dname):
		os.makedirs(dname)

bgfnames = glob.glob(os.path.join(BGIMGDIR, '*.jpg'))

W = 600
H = int(W/1.4142)

blue =   (255, 0, 0, 255)
red =    (0, 0, 255, 255)
green =  (0, 255, 0, 255)
violet = (180, 0, 180, 255)
yellow = (0, 180, 180, 255)
white =  (255, 255, 255, 255)
black =  (0, 0, 0, 255)
transparent = (0, 0, 0, 0)

label_preview_colors = (
	(98, 94, 86, 255),
	(231, 226, 217, 255),
	(190, 169, 117, 255),
	(146, 116, 0, 255),
	(85, 69, 10, 255),
	(255, 222, 38, 255),
	(0, 237, 255, 255),
	(0, 193, 255, 255),
	(0, 124, 213, 255),
	(177, 215, 255, 255),
	(0, 108, 255, 255),
	(62, 71, 86, 255),
)

font = cv2.FONT_HERSHEY_COMPLEX
fontScale = 250
thickness = -1

ft = cv2.freetype.createFreeType2()
ft.loadFontData(fontFileName='Ubuntu-R.ttf', id=0)

def overlay_image_alpha(img, img_overlay, pos, alpha_mask):
	"""Overlay img_overlay on top of img at the position specified by
	pos and blend using alpha_mask.

	Alpha mask must contain values within the range [0, 1] and be the
	same size as img_overlay.
	"""

	rs = img.copy()

	x, y = pos

	# Image ranges
	y1, y2 = max(0, y), min(rs.shape[0], y + img_overlay.shape[0])
	x1, x2 = max(0, x), min(rs.shape[1], x + img_overlay.shape[1])

	# Overlay ranges
	y1o, y2o = max(0, -y), min(img_overlay.shape[0], rs.shape[0] - y)
	x1o, x2o = max(0, -x), min(img_overlay.shape[1], rs.shape[1] - x)

	# Exit if nothing to do
	if y1 >= y2 or x1 >= x2 or y1o >= y2o or x1o >= x2o:
		return

	channels = rs.shape[2]

	alpha = alpha_mask[y1o:y2o, x1o:x2o]
	alpha_inv = 1.0 - alpha

	for c in range(channels):
		rs[y1:y2, x1:x2, c] = (alpha * img_overlay[y1o:y2o, x1o:x2o, c] +
								alpha_inv * rs[y1:y2, x1:x2, c])

	return rs


def overlay_transparent(background, overlay, x, y):

	background = background.copy();

	background_width = background.shape[1]
	background_height = background.shape[0]

	if x >= background_width or y >= background_height:
		return background

	h, w = overlay.shape[0], overlay.shape[1]

	if x + w > background_width:
		w = background_width - x
		overlay = overlay[:, :w]

	if y + h > background_height:
		h = background_height - y
		overlay = overlay[:h]

	if overlay.shape[2] < 4:
		overlay = np.concatenate(
			[
				overlay,
				np.ones((overlay.shape[0], overlay.shape[1], 1), dtype = overlay.dtype) * 255
			],
			axis = 2,
		)

	overlay_image = overlay[..., :3]
	mask = overlay[..., 3:] / 255.0

	print('mask shape', mask.shape)

	background[y:y+h, x:x+w, :3] = (1.0 - mask) * background[y:y+h, x:x+w, :3] + mask * overlay_image[:, :, :3]

	return background

def rotate_bound(image, angle, interpolation = cv2.INTER_NEAREST):
	# grab the dimensions of the image and then determine the
	# center
	(h, w) = image.shape[:2]
	(cX, cY) = (w / 2, h / 2)

	# grab the rotation matrix (applying the negative of the
	# angle to rotate clockwise), then grab the sine and cosine
	# (i.e., the rotation components of the matrix)
	M = cv2.getRotationMatrix2D((cX, cY), -angle, 1.0)
	cos = np.abs(M[0, 0])
	sin = np.abs(M[0, 1])

	# compute the new bounding dimensions of the image
	nW = int((h * sin) + (w * cos))
	nH = int((h * cos) + (w * sin))

	# adjust the rotation matrix to take into account translation
	M[0, 2] += (nW / 2) - cX
	M[1, 2] += (nH / 2) - cY

	# perform the actual rotation and return the image
	# use cv2.INTER_NEAREST to avoid introducing new colour
	return cv2.warpAffine(image, M, (nW, nH), flags=interpolation)


def make_numplate_rotate(num):

	n = 0

	image = np.empty((H, W, 3), dtype=np.uint8)
	image[:] = white[:3]
	label_image = np.zeros_like(image)
	shape = image.shape
#	print('Source shape', shape)

	hspace = 10
	tsize = []
	xsize = 0
	ysize = 0
	for c in str(num):

		textSize = ft.getTextSize(c, fontScale, thickness);
#		print(c, 'textSize(w,h)', textSize)
		tsize.append(textSize)

		xsize += textSize[0][0] + hspace
		ysize = max(ysize, textSize[0][1])

	xpos = (shape[1] - xsize) // 2
	ypos = (shape[0] + ysize) // 2
	ann = []

	for i,c in enumerate(str(num)):

		ft.putText(image, text=c, org=(xpos, ypos), fontHeight=fontScale, color=blue, thickness=thickness, line_type=cv2.LINE_4, bottomLeftOrigin=True)
		ft.putText(img=label_image, text=c, org=(xpos, ypos), fontHeight=fontScale, color=[0, 0, int(c)+1], thickness=thickness, line_type=cv2.LINE_4, bottomLeftOrigin=True)

		xpos += textSize[0][0] + hspace

	image = cv2.cvtColor(image, cv2.COLOR_BGR2BGRA)
	label_image = label_image[:,:,2]

	ofname = '%d.png' % num
	cv2.imwrite(os.path.join(OUTDIR, IMGDIR, ofname), image)
	cv2.imwrite(os.path.join(OUTDIR, LABELDIR, ofname), label_image)

	for i in range(10):
		sz_coef = float(random.randint(3, 6)) / 10
		resized_image = cv2.resize(image, None, fx=sz_coef, fy=sz_coef, interpolation = cv2.INTER_CUBIC)
		resized_label_image = cv2.resize(label_image, None, fx=sz_coef, fy=sz_coef, interpolation = cv2.INTER_NEAREST)
		print('compare', image.shape, resized_image.shape)

		angle = random.randint(1, 360//5) * 5
		rotated_image = rotate_bound(resized_image, angle, interpolation = cv2.INTER_CUBIC)
		rotated_label_image = rotate_bound(resized_label_image, angle, interpolation = cv2.INTER_NEAREST)

		if list(np.unique(label_image)) != list(np.unique(rotated_label_image)):
			print('uniq before:', np.unique(label_image))
			print('uniq after:', np.unique(rotated_label_image))
		else:
			print('uniq labels:', np.unique(label_image))

		# replace black with color to see
#		rotated_image[np.where((rotated_image==black).all(axis=2))] = white
#		rotated_image[np.where((rotated_image==transparent).all(axis=2))] = yellow

#		ret, mask = cv2.threshold(rotated_label_image, 0, 255, cv2.THRESH_BINARY)
#		print('mask shape', mask.shape)
#		mask_inv = cv2.bitwise_not(mask)

#		bg_image = np.empty_like(rotated_image)
#		bg_image[:] = black

		print('rotated_label_image', rotated_label_image.shape, rotated_label_image.dtype)

		dst_image = cv2.imread(random.choice(bgfnames))
		dst_image = cv2.cvtColor(dst_image, cv2.COLOR_BGR2BGRA)
		bg_shape = rotated_image.shape
		bg_xpos = random.randint(0, dst_image.shape[1]-bg_shape[1])
		bg_ypos = random.randint(0, dst_image.shape[0]-bg_shape[0])
		bg_image = dst_image[bg_ypos:bg_ypos+bg_shape[0],bg_xpos:bg_xpos+bg_shape[1]]
		view_through = random.randint(1, 5)

#		print('bg_image', bg_image.shape, bg_image.dtype)

#		image_fg = cv2.bitwise_and(rotated_image, rotated_image, mask = mask)
#		image_bg = cv2.bitwise_and(bg_image, bg_image, mask = mask_inv)

#		print('image_fg', image_fg.shape, image_fg.dtype)
#		print('image_bg', image_bg.shape, image_bg.dtype)

#		image_dst = cv2.add(image_bg, image_fg)

#		dst_image[bg_ypos:bg_ypos+bg_shape[0],bg_xpos:bg_xpos+bg_shape[1]] = image_dst
		#st_image[bg_ypos:bg_ypos+bg_shape[0],bg_xpos:bg_xpos+bg_shape[1]] = cv2.addWeighted(bg_image, 0.5, rotated_image, 0.5, 0)
#		dst_image[bg_ypos:bg_ypos+bg_shape[0],bg_xpos:bg_xpos+bg_shape[1]] = overlay_transparent(bg_image, rotated_image, 0, 0)
		dst_image[bg_ypos:bg_ypos+bg_shape[0],bg_xpos:bg_xpos+bg_shape[1]] = overlay_image_alpha(rotated_image, bg_image, (0, 0), (1-rotated_image[:, :, 3] / 255.0 / view_through))

		big_label_image = np.zeros(dst_image.shape[:2], np.uint8)
		big_label_image[bg_ypos:bg_ypos+bg_shape[0],bg_xpos:bg_xpos+bg_shape[1]] = rotated_label_image

		label_preview_image = np.empty_like(dst_image)
		label_preview_image[:] = white;

		ofname_dst = '%d_r%d_szf_%.2f.png' % (num, angle,sz_coef)

		xml_root = ET.Element('annotation', verified='yes')
		ET.SubElement(xml_root, 'folder').text = os.path.dirname(ofname_dst)
		ET.SubElement(xml_root, 'filename').text = os.path.basename(ofname_dst)
		ET.SubElement(xml_root, 'path').text = ofname_dst
		ET.SubElement(ET.SubElement(xml_root, 'source'), 'database').text = 'unknown'
		xml_el = ET.SubElement(xml_root, 'size')
		ET.SubElement(xml_el, 'width').text = str(dst_image.shape[1])
		ET.SubElement(xml_el, 'height').text = str(dst_image.shape[0])
		ET.SubElement(xml_el, 'depth').text = str(3)
		ET.SubElement(xml_root, 'source').text = 0

		# uniq labels
		for label in np.unique(big_label_image[big_label_image>0]):
			nz = np.nonzero(big_label_image == label)
			ymin = min(nz[0])
			ymax = max(nz[0])
			xmin = min(nz[1])
			xmax = max(nz[1])
			print('rect:', xmin, ymin, xmax, ymax)
#			cv2.rectangle(label_preview_image, (xmin, ymin), (xmax, ymax), label_preview_colors[10-label], cv2.FILLED)
			xml_el = ET.SubElement(xml_root, 'object')
			ET.SubElement(xml_el, 'name').text = str(label)
			ET.SubElement(xml_el, 'pose').text = 'Unspecified'
			ET.SubElement(xml_el, 'truncated').text = str(0)
			ET.SubElement(xml_el, 'difficult').text = str(0)
			xml_el1 = ET.SubElement(xml_el, 'bndbox')
			ET.SubElement(xml_el1, 'xmin').text = str(xmin)
			ET.SubElement(xml_el1, 'ymin').text = str(ymin)
			ET.SubElement(xml_el1, 'xmax').text = str(xmax)
			ET.SubElement(xml_el1, 'ymax').text = str(ymax)


		for i in range(1, 11):
			label_preview_image[np.where((big_label_image==i))] = label_preview_colors[i-1]

#		ret, mask = cv2.threshold(big_label_image, 0, 255, cv2.THRESH_BINARY)
#		mask_inv = cv2.bitwise_not(mask)

#		image_fg = cv2.bitwise_and(dst_image, dst_image, mask = mask)
#		image_bg = cv2.bitwise_and(dst_image, dst_image, mask = mask_inv)


		cv2.imwrite(os.path.join(OUTDIR, IMGDIR, ofname_dst), cv2.cvtColor(dst_image, cv2.COLOR_BGRA2BGR))
		cv2.imwrite(os.path.join(OUTDIR, LABELDIR, ofname_dst), big_label_image)
		cv2.imwrite(os.path.join(OUTDIR, LABELPREVIEWDIR, ofname_dst), label_preview_image)
		open(os.path.join(os.path.join(OUTDIR, XML_LOCATION), os.path.basename(ofname_dst)) + '.xml', 'w').write(minidom.parseString(ET.tostring(xml_root)).toprettyxml(indent="\t"))
		n += 1
	return n

for i in range(100):
	n = make_numplate_rotate(random.randint(100, 999))
	print('Made %d' % n)

print('Finished')
