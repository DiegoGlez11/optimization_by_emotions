EEGNet, DeepConvNet, SHallowConvNet


Model: "model_1"
_________________________________________________________________
 Layer (type)                Output Shape              Param #   
=================================================================
 input_2 (InputLayer)        [(None, 16, 128, 1)]      0         
                                                                 
 conv2d_5 (Conv2D)           (None, 16, 128, 8)        256       
                                                                 
 batch_normalization_4 (Batc  (None, 16, 128, 8)       32        
 hNormalization)                                                 
                                                                 
 depthwise_conv2d (Depthwise  (None, 1, 128, 16)       256       
 Conv2D)                                                         
                                                                 
 batch_normalization_5 (Batc  (None, 1, 128, 16)       64        
 hNormalization)                                                 
                                                                 
 activation_5 (Activation)   (None, 1, 128, 16)        0         
                                                                 
 average_pooling2d (AverageP  (None, 1, 32, 16)        0         
 ooling2D)                                                       
                                                                 
 dropout_4 (Dropout)         (None, 1, 32, 16)         0         
                                                                 
 separable_conv2d (Separable  (None, 1, 32, 16)        512       
 Conv2D)                                                         
                                                                 
 batch_normalization_6 (Batc  (None, 1, 32, 16)        64        
 hNormalization)                                                 
                                                                 
 activation_6 (Activation)   (None, 1, 32, 16)         0         
                                                                 
 average_pooling2d_1 (Averag  (None, 1, 4, 16)         0         
 ePooling2D)                                                     
                                                                 
 dropout_5 (Dropout)         (None, 1, 4, 16)          0         
                                                                 
 flatten (Flatten)           (None, 64)                0         
                                                                 
 dense (Dense)               (None, 2)                 130       
                                                                 
 activation_7 (Activation)   (None, 2)                 0         
                                                                 
=================================================================
Total params: 1,314
Trainable params: 1,234
Non-trainable params: 80
_________________________________________________________________
Model: "model_2"
_________________________________________________________________
 Layer (type)                Output Shape              Param #   
=================================================================
 input_3 (InputLayer)        [(None, 16, 128, 1)]      0         
                                                                 
 conv2d_6 (Conv2D)           (None, 16, 124, 25)       150       
                                                                 
 conv2d_7 (Conv2D)           (None, 1, 124, 25)        10025     
                                                                 
 batch_normalization_7 (Batc  (None, 1, 124, 25)       100       
 hNormalization)                                                 
                                                                 
 activation_8 (Activation)   (None, 1, 124, 25)        0         
                                                                 
 max_pooling2d_4 (MaxPooling  (None, 1, 62, 25)        0         
 2D)                                                             
                                                                 
 dropout_6 (Dropout)         (None, 1, 62, 25)         0         
                                                                 
 conv2d_8 (Conv2D)           (None, 1, 58, 50)         6300      
                                                                 
 batch_normalization_8 (Batc  (None, 1, 58, 50)        200       
 hNormalization)                                                 
                                                                 
 activation_9 (Activation)   (None, 1, 58, 50)         0         
                                                                 
 max_pooling2d_5 (MaxPooling  (None, 1, 29, 50)        0         
 2D)                                                             
                                                                 
 dropout_7 (Dropout)         (None, 1, 29, 50)         0         
                                                                 
 conv2d_9 (Conv2D)           (None, 1, 25, 100)        25100     
                                                                 
 batch_normalization_9 (Batc  (None, 1, 25, 100)       400       
 hNormalization)                                                 
                                                                 
 activation_10 (Activation)  (None, 1, 25, 100)        0         
                                                                 
 max_pooling2d_6 (MaxPooling  (None, 1, 12, 100)       0         
 2D)                                                             
                                                                 
 dropout_8 (Dropout)         (None, 1, 12, 100)        0         
                                                                 
 conv2d_10 (Conv2D)          (None, 1, 8, 200)         100200    
                                                                 
 batch_normalization_10 (Bat  (None, 1, 8, 200)        800       
 chNormalization)                                                
                                                                 
 activation_11 (Activation)  (None, 1, 8, 200)         0         
                                                                 
 max_pooling2d_7 (MaxPooling  (None, 1, 4, 200)        0         
 2D)                                                             
                                                                 
 dropout_9 (Dropout)         (None, 1, 4, 200)         0         
                                                                 
 flatten_1 (Flatten)         (None, 800)               0         
                                                                 
 dense_1 (Dense)             (None, 2)                 1602      
                                                                 
 activation_12 (Activation)  (None, 2)                 0         
                                                                 
=================================================================
Total params: 144,877
Trainable params: 144,127
Non-trainable params: 750
_________________________________________________________________
Model: "model_3"
_________________________________________________________________
 Layer (type)                Output Shape              Param #   
=================================================================
 input_4 (InputLayer)        [(None, 16, 128, 1)]      0         
                                                                 
 conv2d_11 (Conv2D)          (None, 16, 116, 40)       560       
                                                                 
 conv2d_12 (Conv2D)          (None, 1, 116, 40)        25600     
                                                                 
 batch_normalization_11 (Bat  (None, 1, 116, 40)       160       
 chNormalization)                                                
                                                                 
 activation_13 (Activation)  (None, 1, 116, 40)        0         
                                                                 
 average_pooling2d_2 (Averag  (None, 1, 12, 40)        0         
 ePooling2D)                                                     
                                                                 
 activation_14 (Activation)  (None, 1, 12, 40)         0         
                                                                 
 dropout_10 (Dropout)        (None, 1, 12, 40)         0         
                                                                 
 flatten_2 (Flatten)         (None, 480)               0         
                                                                 
 dense_2 (Dense)             (None, 2)                 962       
                                                                 
 activation_15 (Activation)  (None, 2)                 0         
                                                                 
=================================================================
Total params: 27,282
Trainable params: 27,202
Non-trainable params: 80
_________________________________________________________________
