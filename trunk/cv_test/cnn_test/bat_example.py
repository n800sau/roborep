import os
from batcountry import BatCountry
import numpy as np
from PIL import Image

bc = BatCountry(os.path.expanduser("~/install/caffe/models/bvlc_googlenet"))
image = bc.dream(np.float32(Image.open("cat.1.jpg")))
bc.cleanup()

result = Image.fromarray(np.uint8(image))
result.save("output.jpg")
