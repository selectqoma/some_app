# slagsquare\_pot\_instance\_segmentation

In order to use this package you must first download the Mask_RCNN repository and
append its location to the PYTHONPATH
```bash
$ git clone https://github.com/matterport/Mask_RCNN ~/Mask_RCNN
$ echo 'export MASK_RCNN_ROOT_DIR="~/Mask_RCNN"'
$ echo "export POT_RGB_INSTANCE_SEGMENTATION_DATASET_ROOT_DIR="~/pot_rgb_instance_segmentation_dataset_root_dir"
$ echo '[[ ":$PYTHONPATH:" != *":$MASK_RCNN_ROOT_DIR:"* ]] && PYTHONPATH="$PYTHONPATH:$MASK_RCNN_ROOT_DIR"' >> ~/.bashrc
```
