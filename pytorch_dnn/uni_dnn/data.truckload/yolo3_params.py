TRAINING_PARAMS = \
{
	"model_params": {
		"backbone_name": "darknet_53"
	},
	"anchors": [[[116, 90], [156, 198], [373, 326]],
				[[30, 61], [62, 45], [59, 119]],
				[[10, 13], [16, 30], [33, 23]]],
	"lr": {
		"backbone_lr": 0.001,
		"other_lr": 0.01,
		"freeze_backbone": False,   #  freeze backbone wegiths to finetune
		"decay_gamma": 0.1,
		"decay_step": 20,		   #  decay lr in every ? epochs
	},
	"optimizer": {
		"type": "sgd",
		"weight_decay": 4e-05,
	},
	"img_h": 416,
	"img_w": 416,
	"parallels": [0],						 #  config GPU device
	"confidence_threshold": 0.80,
}
