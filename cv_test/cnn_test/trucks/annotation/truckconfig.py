from PyQt4.QtGui import QPen
from PyQt4.Qt import Qt
from sloth.items import PolygonItem


class DirtItem(PolygonItem):

    def __init__(self, *args, **kwargs):
        PolygonItem.__init__(self, *args, **kwargs)
        # set drawing pen to red with width 2
        self.setPen(QPen(Qt.green, 2))

LABELS = (
    {
        'attributes': {
            'class':    'dirt',
        },
        'inserter': 'sloth.items.PolygonItemInserter',
        'item':     DirtItem,
        'hotkey':   'd',
        'text':     'Dirt',
    },
    {
        'attributes': {
            'class':    'belaz',
        },
        'inserter': 'sloth.items.PolygonItemInserter',
        'item':     'sloth.items.PolygonItem',
        'hotkey':   'b',
        'text':     'Belaz',
    },
)

