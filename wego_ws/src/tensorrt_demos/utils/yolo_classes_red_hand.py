COCO_CLASSES_LIST = [
    'red_hand'
]

def get_cls_dict(category_num):
    """Get the class ID to name translation dictionary."""
    if category_num == 1:
        return {i: n for i, n in enumerate(COCO_CLASSES_LIST)}
    else:
        return {i: 'CLS%d' % i for i in range(category_num)}
