import tensorflow as tf

def channel_wise_attention(feature_map, H, W, C, weight_decay=0.00004, scope='', reuse=None):
    """This method is used to add spatial attention to model.
    
    Parameters
    ---------------
    @feature_map: Which visual feature map as branch to use.
    @K: Map `H*W` units to K units. Now unused.
    @reuse: reuse variables if use multi gpus.
    
    Return
    ---------------
    @attended_fm: Feature map with Channel-Wise Attention.
    """
    with tf.variable_scope(scope, 'ChannelWiseAttention', reuse=reuse):
        # Tensorflow's tensor is in BHWC format. H for row split while W for column split.
        # _, H, W, C = tuple([int(x) for x in feature_map.get_shape()])
        weight = tf.get_variable("weight", [C, C],
                              dtype=tf.float32,
                              initializer=tf.initializers.orthogonal,
                              regularizer=tf.contrib.layers.l2_regularizer(weight_decay))
        bias = tf.get_variable("bias", [C],
                              dtype=tf.float32,
                              initializer=tf.initializers.zeros)

        transpose_feature_map = tf.transpose(tf.reduce_mean(feature_map, [1, 2], keep_dims=True),
                                             perm=[0, 3, 1, 2])
        channel_wise_attention_fm = tf.matmul(tf.reshape(transpose_feature_map, 
                                                         [-1, C]), weight) + bias
        channel_wise_attention_fm = tf.nn.sigmoid(channel_wise_attention_fm)
#         channel_wise_attention_fm = tf.clip_by_value(tf.nn.relu(channel_wise_attention_fm), 
#                                                      clip_value_min = 0, 
#                                                      clip_value_max = 1)
        attention = tf.reshape(tf.concat([channel_wise_attention_fm] * (H * W),
                                         axis=1), [-1, H, W, C])
        attended_fm = attention * feature_map
        return attended_fm