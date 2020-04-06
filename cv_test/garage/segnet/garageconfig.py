from PyQt4.QtGui import QPen
from PyQt4.Qt import Qt
from sloth.items import PolygonItem


class CarItem(PolygonItem):

    def __init__(self, *args, **kwargs):
        PolygonItem.__init__(self, *args, **kwargs)

        # set drawing pen to red with width 2
        self.setPen(QPen(Qt.yellow, 2))

LABELS = (
    {
        'attributes': {
            'class':    'car',
        },
        'inserter': 'sloth.items.PolygonItemInserter',
        'item':     CarItem,
        'hotkey':   'c',
        'text':     'Car',
    },
)

