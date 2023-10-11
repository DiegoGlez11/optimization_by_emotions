#python3
import numpy as np
import pandas as pd
import tensorflow as tf
from cnn_class import cnn
import time
import scipy.io as sio
import pickle
import math
from Attention.channel_attention import  channel_wise_attention
from Attention.disan import directional_attention_with_dense, multi_dimensional_attention
import os

#the window length of EEG sample
window_size = 384
# the channel of EEG sample, DEAP:32 DREAMER:14
n_channel = 32


def deap_preprocess(data_file,dimention, dir_in):
	with_or_without = 'yes'
	if type(dir_in) == type(None):
		# set the file type and path
		rnn_suffix = "._dataset.pkl"
		label_suffix = "._labels.pkl"
		arousal_or_valence = dimention
		
		dataset_dir = "/home/taozi12345/deap_shuffled_data_3s/" + with_or_without + "_" + arousal_or_valence + "/"
	else:
		# set the file type and path
		rnn_suffix = ".mat_win_384_rnn_dataset.pkl"
		label_suffix = ".mat_win_384_labels.pkl"
		dataset_dir = dir_in
	
	
	with open(dataset_dir + data_file + rnn_suffix, "rb") as fp:
		rnn_datasets = pickle.load(fp)
	with open(dataset_dir + data_file + label_suffix, "rb") as fp:
		labels = pickle.load(fp)
		labels = np.transpose(labels)
    # print("loaded shape:",labels.shape)
	lables_backup = labels
	one_hot_labels = np.array(list(pd.get_dummies(labels)))

	labels = np.asarray(pd.get_dummies(labels), dtype=np.int8)

	# shuffle data
	index = np.array(range(0, len(labels)))
	np.random.shuffle(index)
	rnn_datasets = rnn_datasets[index]  # .transpose(0,2,1)
	labels = labels[index]
	datasets = rnn_datasets

	datasets = datasets.reshape(-1, 384, 14, 1).astype('float32')
	labels = labels.astype('float32')
	return datasets , labels

def process_deap(dir_in=None):
	###########################################################################
	# set model parameters
	###########################################################################
	# kernel parameter
	kernel_height_1st = 32 #DREAMER：14
	kernel_width_1st = 45
	kernel_stride = 1
	conv_channel_num = 40
	# pooling parameter
	pooling_height_1st = 1
	pooling_width_1st = 75
	pooling_stride_1st = 10
	# full connected parameter
	attention_size = 512
	n_hidden_state = 64
	###########################################################################
	# input channel
	input_channel_num = 1
	# input height
	input_height = 32 #DREAMER：14
	# input width
	input_width = 384
	# prediction class
	num_labels = 2
	###########################################################################
	# set training parameters
	###########################################################################
	# step length
	num_timestep = 1
	# set learning rate
	learning_rate = 1e-4
	# set maximum traing epochs
	training_epochs = 200
	# set batch size
	batch_size = 10
	# set dropout probability
	dropout_prob = 0.5
	# instance cnn class
	padding = 'VALID'
	cnn_2d = cnn(padding=padding)

	################################## The ACRNN Model #########################################
	# input placeholder
	X = tf.placeholder(tf.float32, shape=[None, input_height, input_width, input_channel_num], name = 'X')
	Y = tf.placeholder(tf.float32, shape=[None, num_labels], name = 'Y')
	train_phase = tf.placeholder(tf.bool, name = 'train_phase')
	keep_prob = tf.placeholder(tf.float32, name='keep_prob')

	# channel-wise attention layer
	X_1 = tf.transpose(X,[0, 3, 2, 1])
	conv = channel_wise_attention(X_1, 1, window_size, n_channel, weight_decay=0.00004, scope='', reuse=None)
	conv_1 = tf.transpose(conv,[0, 3, 2, 1])

	# CNN layer
	conv_1 = cnn_2d.apply_conv2d(conv_1, kernel_height_1st, kernel_width_1st, input_channel_num, conv_channel_num, kernel_stride, train_phase)
	print("conv 1 shape: ", conv_1.get_shape().as_list())
	pool_1 = cnn_2d.apply_max_pooling(conv_1, pooling_height_1st, pooling_width_1st, pooling_stride_1st)
	print("pool 1 shape: ", pool_1.get_shape().as_list())
	pool_1_shape = pool_1.get_shape().as_list()
	pool1_flat = tf.reshape(pool_1, [-1, pool_1_shape[1]*pool_1_shape[2]*pool_1_shape[3]])
	fc_drop = tf.nn.dropout(pool1_flat, keep_prob)

	# LSTMs layer
	lstm_in = tf.reshape(fc_drop, [-1, num_timestep, pool_1_shape[1]*pool_1_shape[2]*pool_1_shape[3]])
	cells = []
	for _ in range(2):
		cell = tf.contrib.rnn.BasicLSTMCell(n_hidden_state, forget_bias=1.0, state_is_tuple=True)
		cell = tf.contrib.rnn.DropoutWrapper(cell, output_keep_prob=keep_prob)
		cells.append(cell)
		lstm_cell = tf.contrib.rnn.MultiRNNCell(cells)

		init_state = lstm_cell.zero_state(batch_size, dtype=tf.float32)

		# output ==> [batch, step, n_hidden_state]
		rnn_op, states = tf.nn.dynamic_rnn(lstm_cell, lstm_in, initial_state=init_state, time_major=False)

	#self-attention layer
	with tf.name_scope('Attention_layer'):
		attention_op = multi_dimensional_attention(rnn_op, 64, scope=None,
												keep_prob=1., is_train=None, wd=0., activation='elu',
												tensor_dict=None, name=None)

		attention_drop = tf.nn.dropout(attention_op, keep_prob)

		y_ = cnn_2d.apply_readout(attention_drop, rnn_op.shape[2].value, num_labels)

	# softmax layer: probability prediction
	y_prob = tf.nn.softmax(y_, name = "y_prob")

	# class prediction
	y_pred = tf.argmax(y_prob, 1, name = "y_pred")

	# cross entropy cost function
	cost = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=y_, labels=Y), name = 'loss')
	update_ops = tf.get_collection(tf.GraphKeys.UPDATE_OPS)
	with tf.control_dependencies(update_ops):
		# set training SGD optimizer
		optimizer = tf.train.AdamOptimizer(learning_rate).minimize(cost)

	# get correctly predicted object
	correct_prediction = tf.equal(tf.argmax(tf.nn.softmax(y_), 1), tf.argmax(Y, 1))

	# calculate prediction accuracy
	accuracy = tf.reduce_mean(tf.cast(correct_prediction, tf.float32), name = 'accuracy')

	##########################################################################################

	############################Experiments on database##############################

	#subjects of databases
	deap_subjects = ['s01', 's02', 's03', 's04', 's05', 's06', 's07', 's08', 's09', 's10', 's11','s12', 's13','s14','s15', 's16', 's17',
				's18', 's19', 's20', 's21', 's22', 's23', 's24', 's25', 's26',
			's27', 's28', 's29', 's30', 's31', 's32']

	dreamer_subjects = ['1','2','3','4','5','6','7','8','9','10','11','12','13',
				'14','15','16','17','18','19','20','21','22','23']
				

	for subject in subjects:
	# load one subject's data and labels on arousal on DEAP
		dimention = 'arousal' # 'valence'/'dominance'
		datasets, labels = deap_preprocess(subject, dimention, dir_in)
		
	# load one subject's data and labels on arousal on Dreamer
	# datasets, labels = dreamer_preprocess(subject)
	# devide the data and label into training ets and testing sets based on 10-fold crosss-validation  
		fold = 10
		test_accuracy_all_fold = np.zeros(shape=[0], dtype=float)
		
	# compting in each fold '0-9'    
		for curr_fold in range(fold):
			fold_size = datasets.shape[0]//fold
			indexes_list = [i for i in range(len(datasets))]
			indexes = np.array(indexes_list)
			split_list = [i for i in range(curr_fold*fold_size,(curr_fold+1)*fold_size)]
			split = np.array(split_list)	
			test_y = labels[split]
			test_x = datasets[split]
			split = np.array(list(set(indexes_list)^set(split_list)))
			train_x = datasets[split]
			train_y = labels[split]
			train_x = np.transpose(train_x, [0, 2, 1, 3])
			test_x = np.transpose(test_x, [0, 2, 1, 3])
			
			# run with gpu memory growth
			os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
			os.environ["CUDA_VISIBLE_DEVICES"] = "1"
			config = tf.ConfigProto()
			config.gpu_options.allow_growth = True
			
			# set train batch number per epoch
			batch_num_per_epoch = train_x.shape[0] // batch_size
			train_acc = []
			test_acc = []
			best_test_acc = []
			train_loss = []
			with tf.Session(config=config) as session:
				session.run(tf.global_variables_initializer())
				summary_writer = tf.summary.FileWriter('./log/', session.graph)
				best_acc = 0
				for epoch in range(training_epochs):
					pred_test = np.array([])
					true_test = []
					prob_test = []
					cost_history = np.zeros(shape=[0], dtype=float)
					# training process
					for b in range(batch_num_per_epoch):
						offset = (b * batch_size) % (train_y.shape[0] - batch_size)
						batch_x = train_x[offset:(offset + batch_size), :, :, :]
						batch_x = batch_x.reshape([len(batch_x)*num_timestep, n_channel, window_size, 1])
						batch_y = train_y[offset:(offset + batch_size), :]
						_, c = session.run([optimizer, cost], feed_dict={X: batch_x, Y: batch_y, keep_prob: 1-dropout_prob, train_phase: False})
						cost_history = np.append(cost_history, c)
					# calculate train and test accuracy after each training epoch
					if(epoch%1 == 0):
						train_accuracy 	= np.zeros(shape=[0], dtype=float)
						test_accuracy	= np.zeros(shape=[0], dtype=float)
						test_loss = np.zeros(shape=[0], dtype=float)
						train_l = np.zeros(shape=[0], dtype=float)
						test_l = np.zeros(shape=[0], dtype=float)
						print('(' + time.asctime(time.localtime(time.time())) + ')' + '-' * 30 + ' subject ' + subject + ' fold  ' + str(
							curr_fold) + '  Begin: train' + '-' * 30)
						train_begin_time = time.time()
						# calculate train accuracy after each training epoch
						for i in range(batch_num_per_epoch):
							offset = (i * batch_size) % (train_y.shape[0] - batch_size)
							train_batch_x = train_x[offset:(offset + batch_size), :, :, :]
							train_batch_x = train_batch_x.reshape([len(train_batch_x)*num_timestep, n_channel, window_size, 1])
							train_batch_y = train_y[offset:(offset + batch_size), :]
							train_a, train_c = session.run([accuracy, cost], feed_dict={X: train_batch_x, Y: train_batch_y, keep_prob: 1.0, train_phase: False})
							train_l = np.append(train_l, train_c)
							train_accuracy = np.append(train_accuracy, train_a)
						train_acc = train_acc + [np.mean(train_accuracy)]
						train_loss = train_loss + [np.mean(train_l)]
						
						# calculate test accuracy after each training epoch
						for j in range(batch_num_per_epoch):
							offset = (j * batch_size) % (test_y.shape[0] - batch_size)
							test_batch_x = test_x[offset:(offset + batch_size), :, :, :]
							test_batch_x = test_batch_x.reshape([len(test_batch_x)*num_timestep, n_channel, window_size, 1])
							test_batch_y = test_y[offset:(offset + batch_size), :]

							test_a, test_c, prob_v, pred_v = session.run([accuracy, cost, y_prob, y_pred], feed_dict={X: test_batch_x, Y: test_batch_y, keep_prob: 1.0, train_phase: True})

							test_accuracy = np.append(test_accuracy, test_a)
							test_loss = np.append(test_loss, test_c)

						print("("+time.asctime(time.localtime(time.time()))+") Epoch: ", epoch+1, " Test Cost: ", np.mean(test_loss), "Test Accuracy: ", np.mean(test_accuracy))

				test_accuracy_batch_num = math.floor(test_x.shape[0] / batch_size)
				train_accuracy_save = np.zeros(shape=[0], dtype=float)
				test_accuracy_save = np.zeros(shape=[0], dtype=float)
				test_loss_save = np.zeros(shape=[0], dtype=float)
				for k in range(test_accuracy_batch_num):
					start = k * batch_size
					offset = batch_size
					test_rnn_batch = test_x[start:(start + offset), :, :]
					test_batch_y = test_y[start:(start + offset), :]
					test_a, test_c, prob_v, pred_v = session.run([accuracy, cost, y_prob, y_pred],
														feed_dict={X: test_rnn_batch, Y: test_batch_y, keep_prob: 1.0,
																	train_phase: False})
																	
					test_accuracy_save = np.append(test_accuracy_save, test_a)
					test_loss_save = np.append(test_loss_save, test_c)
				print("Final Test Cost: ", np.mean(test_loss_save), "Final Test Accuracy: ", np.mean(test_accuracy_save))

				test_accuracy_all_fold = np.append(test_accuracy_all_fold, np.mean(test_accuracy_save))

		summary = pd.DataFrame({'fold': range(1, fold+1), 'test_accuracy': test_accuracy_all_fold})
		writer = pd.ExcelWriter("./result_att_cnn_rnn_att_deap/"+ dimention +"/"+ subject +"_"+"summary"+".xlsx")
		summary.to_excel(writer, 'summary', index=False)
		writer.save()




















