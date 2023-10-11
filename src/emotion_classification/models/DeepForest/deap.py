from GCForest import gcForest
import _pickle as pickle
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score
import numpy as np
import datetime
import pandas as pd


starttime = datetime.datetime.now()

file_list=['s01','s02','s03','s04','s05','s06','s07','s08','s09','s10','s11','s12','s13','s14','s15','s16','s17','s18','s19','s20','s22', 's23','s24', 's25', 's21', 's26', 's27', 's28', 's29', 's30','s31','s32' ] #
file_list = ["s22"]
test_accuracy_all_sub=np.zeros(shape=[0],dtype=float)
mean_accuracy_all=0
for data_file in file_list:
    print('sub:', data_file)
    rnn_suffix = ".dat_dataset.pkl"
    label_suffix = ".dat_labels.pkl"
    arousal_or_valence = 'valence'
    dataset_dir = "./deap_shuffled_data/" + arousal_or_valence + "/"
    ###load training set
    data = pickle.load(open(dataset_dir + data_file + rnn_suffix, 'rb'), encoding='utf-8')
    label = pickle.load(open(dataset_dir + data_file + label_suffix, 'rb'), encoding='utf-8')

    X = data
    y = label

    fold = 10
    count = 0
    test_accuracy_all_fold = np.zeros(shape=[0], dtype=float)
    mean_accuracy = 0
    for curr_fold in range(fold):
        fold_size = X.shape[0] // fold
        indexes_list = [i for i in range(len(X))]
        indexes = np.array(indexes_list)
        split_list = [i for i in range(curr_fold * fold_size, (curr_fold + 1) * fold_size)]
        print("split", len(split_list))
        split = np.array(split_list)

        X_te = X[split]
        y_te = y[split]
        print("data split:",X_te.shape, y_te.shape)

        split = np.array(list(set(indexes_list) ^ set(split_list)))
        X_tr = X[split]
        y_tr = y[split]
        print(count)
        print("train_x shape:", X_tr.shape)
        print("test_x shape:", X_te.shape)

        train_sample = y_tr.shape[0]
        test_sample = y_te.shape[0]
        #print("1. x_tr",X_tr.shape,"x_te",X_te.shape)
        #1. x_tr (2160, 128, 9, 9) x_te (240, 128, 9, 9)

        X_tr = X_tr.transpose(1, 0, 2, 3)
        X_te = X_te.transpose(1, 0, 2, 3)
        #print("2. x_tr",X_tr.shape,"x_te",X_te.shape)
        #2. x_tr (128, 2160, 9, 9) x_te (128, 240, 9, 9)

        print("ini",X_tr.shape)
        X_tr_ = X_tr[0]
        print("xtr[0]", X_tr_.shape)
        X_tr_ = X_tr_.reshape(X_tr.shape[1], 81)
        print("reshape xtr[0]", X_tr_.shape)
        gcf = gcForest(shape_1X=[9, 9], window=6, tolerance=0.0, min_samples_mgs=20, min_samples_cascade=10)
        print("scanning tr: ", X_tr_.shape, y_tr.shape)
        #scanning tr:  (2160, 81) (2160,)

        X_tr_mgs_ = gcf.mg_scanning(X_tr_, y_tr)
        X_te_ = X_te[0]
        X_te_ = X_te_.reshape(X_te.shape[1], 81)
        print("scanning te: ", X_te_.shape)
        #scanning te:  (240, 81)

        X_te_mgs_ = gcf.mg_scanning(X_te_)

        X_tr_mgs = X_tr_mgs_
        X_te_mgs = X_te_mgs_

        for i in range(126, X_tr.shape[0]):
            X_tr_ = X_tr[i]
            X_tr_ = X_tr_.reshape(X_tr.shape[1], 81)
            gcf = gcForest(shape_1X=[9, 9], window=6, tolerance=0.0, min_samples_mgs=20, min_samples_cascade=10)
            X_tr_mgs_ = gcf.mg_scanning(X_tr_, y_tr)
            X_tr_mgs = np.concatenate((X_tr_mgs, X_tr_mgs_), axis=1)
            #print('X_tr_mgs_.shape:', X_tr_mgs_.shape)
            X_te_ = X_te[i]
            X_te_ = X_te_.reshape(X_te.shape[1], 81)
            #print('X_te_mgs_.shape:', X_te_mgs_.shape)
            X_te_mgs_ = gcf.mg_scanning(X_te_)
            X_te_mgs = np.concatenate((X_te_mgs, X_te_mgs_), axis=1)
        print("y_tr unique",set(y_tr))
        print('X_te_mgs.shape:', X_te_mgs.shape)
        print('X_tr_mgs.shape:', X_tr_mgs.shape)

        _ = gcf.cascade_forest(X_tr_mgs, y_tr)
        s_ = np.array(_)
        print("cascade", s_.shape)
        c_ = gcf.cascade_forest(X_te_mgs)
        c_ = np.array(c_)
        print("c_", c_.shape)
        pred_proba = np.mean(c_, axis=0)
        print("pred_proba",pred_proba.shape)
        
        # cascade (8, 1944, 2)
        # c_ (8, 240, 2)
        # pred_proba (240, 2)
        # preds (240,)


        # print(X_te_mgs.shape)
        preds = np.argmax   (pred_proba, axis=1)
        print("preds",preds.shape)

        test_sample = y_te.shape[0]
        print("y_te", y_te.shape)
        # evaluating accuracy
        accuracy = accuracy_score(y_true=y_te, y_pred=preds)
        print('gcForest accuracy : {}'.format(accuracy))
        test_accuracy_all_fold = np.append(test_accuracy_all_fold, accuracy)
        mean_accuracy += accuracy
        count += 1

    print(mean_accuracy / fold)
    summary = pd.DataFrame({'fold': range(1, fold + 1), 'test_accuracy': test_accuracy_all_fold})
    writer = pd.ExcelWriter(
        "./result/DEAP/" + arousal_or_valence + "/" + data_file  + ".xlsx")
    summary.to_excel(writer, 'summary', index=False)
    writer.save()
    mean_accuracy_all += mean_accuracy / fold
    test_accuracy_all_sub = np.append(test_accuracy_all_sub, mean_accuracy / fold)
print('mean_accuracy_all:', mean_accuracy_all / 32)
result = pd.DataFrame({'sub': range(1, 33), 'test_accuracy': test_accuracy_all_sub})
writer = pd.ExcelWriter("./result/DEAP/" + arousal_or_valence + "/" + ".xlsx")
result.to_excel(writer, 'result', index=False)
writer.save()

endtime = datetime.datetime.now()
print(endtime - starttime)