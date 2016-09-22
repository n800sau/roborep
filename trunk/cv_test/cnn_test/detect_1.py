import os, json, cv2, glob
from keras.models import model_from_json
from keras.preprocessing import image
import numpy as np

# dimensions of our images.
img_width, img_height = 150, 150

prefix = 'robo_'
weights_fname = prefix + 'weights.h5'

model = model_from_json(file(prefix + 'model.json').read())

model.compile(loss='binary_crossentropy',
              optimizer='rmsprop',
              metrics=['accuracy'])

labels = dict([(v,k) for k,v in json.load(file(prefix + 'labels.json')).items()])

print labels

model.load_weights(prefix + 'weights.h5')

for ifname in glob.glob('data/validation/norobo/*.jpg'):

#ifname = 'data/train/norobo/norobo001.jpg'


	img = image.load_img(ifname, target_size=(150, 150))
#	img.save(os.path.expanduser("~/public_html/test_img.jpg"))
	data = image.img_to_array(img)

	data = np.expand_dims(data, axis=0)
#	print('Input image shape:', data.shape)

#image = cv2.imread(ifname)

#data = cv2.resize(image, (150, 150))


#hdata = np.empty((1, data.shape[2], data.shape[0], data.shape[1]), np.uint8)

#data = np.swapaxes(data, 0, 2)

#hdata[0] = data

	proba = model.predict(data)

	print('PROBA:', proba)

	predictions = proba.argmax(axis=1)

	print('PREDICTIONS:', predictions)

	predict_label = labels[predictions[0]]

	print predict_label

print 'Finished'
