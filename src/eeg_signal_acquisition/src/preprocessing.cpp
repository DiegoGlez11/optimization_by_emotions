#include "ros/ros.h"
#include "neurocontroller_database/get_type_id.h"
#include "eeg_signal_acquisition/eeg_block.h"
#include "eeg_signal_acquisition/get_preprocessing_values.h"
#include "eeg_signal_acquisition/set_preprocessing_values.h"
#include "eeg_signal_acquisition/control_acquisition.h"
#include "eeg_signal_acquisition/eeg_block_srv.h"
#include <vector>
#include <stdlib.h>
#include <unistd.h>
#include <typeinfo>

#include <pwd.h>
#include <iostream>
#include <fstream>

#include "board_shim.h"
#include "data_filter.h"

using namespace std;

// banderas de control en el preprocesamiento
bool do_bandpass = true;
bool do_erase_noise = true;
bool do_detrend = true;
bool do_reference_eeg = true;
bool do_reference_eeg_common = false;
// guardado de archivos
bool is_save = true;
// rango de frecuencias del filtro
int freq_ini = 5;
int freq_end = 50;
// orden del filtro
int filter_order = 4;
ofstream file_eeg;

vector<double> eeg_data;
ros::Publisher topic_prepro;
ros::Publisher topic_prepro_train;
ros::Publisher topic_gui;

int num_channels, num_samples;
// , sampling_rate;

string get_homedir()
{
    struct passwd *pw = getpwuid(getuid());
    string dir(pw->pw_dir);
    return dir;
}

// RUTA RAIZ
std::string root = get_homedir() + "/catkin_ws/src/neurocontroller_database/database";

int get_board_id()
{
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<eeg_signal_acquisition::control_acquisition>("control_acquisition");
    eeg_signal_acquisition::control_acquisition srv_ctl;

    srv_ctl.request.board_id = true;

    if (client.call(srv_ctl))
    {
        return srv_ctl.response.board_id;
    }
    else
    {
        return 0;
    }
}

string get_type_acquisition(string id)
{
    ros::NodeHandle n;
    ros::ServiceClient c = n.serviceClient<neurocontroller_database::get_type_id>("get_type_id");

    neurocontroller_database::get_type_id p_id;
    p_id.request.id_pareto_front = id;

    if (c.call(p_id))
    {
        // cout << "type: " << p_id.response.type << endl;
        return p_id.response.type;
    }
    else
    {
        ROS_ERROR("Failed to call service add_two_ints");
        return "unknown";
    }
}

vector<double> preprocessing_data(vector<double> labels, vector<double> data_eeg, int num_channels, int num_samples, int sampling_rate, string user_name, string id_pareto_front, int num_ind, string id_eeg_signal, int storage_pos, bool is_start, bool is_stop, string type_signal, int time_capturing)
// vector<double> preprocessing_data(vector<double> labels, int num_channels, int num_samples, int sampling_rate, string user_name, string id_pareto_front, int num_ind, string id_eeg_signal, int storage_pos, bool is_start, bool is_stop, string type_signal, int time_capturing)
{
    cout << "------ preprocessing -----" << endl;
    cout << id_pareto_front << " " << num_ind << " " << id_eeg_signal << " " << storage_pos << endl;
    string dir_ind = root + "/" + user_name;

    std::size_t f_b = id_pareto_front.find("optimized_basal");
    if (f_b != std::string::npos)
        dir_ind = dir_ind + "/" + id_pareto_front;
    else
        dir_ind = dir_ind + "/" + get_type_acquisition(id_pareto_front) + "/" + id_pareto_front + "/ind_" + to_string(num_ind);

    // dir de las señales
    string dir_eeg = dir_ind + "/" + id_eeg_signal + ".csv";
    cout << "Dir " << dir_eeg << endl;
    // se cargan las señales
    BrainFlowArray<double, 2> data = DataFilter::read_file(dir_eeg);
    // shape
    num_samples = data.get_size(1);
    num_channels = data.get_size(0);
    num_channels = 16;

    cout << "shape EEG " << num_channels << " * " << num_samples << endl;

    // id
    // int id_board = get_board_id();
    // cout << "id_board " << id_board << endl;
    // // eeg channels
    // vector<int> eeg_channels = BoardShim::get_eeg_channels(id_board);

    // referencia EEG
    vector<double> ref_eeg(num_samples, 0);

    cout << "id_pareto_front: " << id_pareto_front << " num_ind: " << num_ind << " channels:" << num_channels << " size: " << num_samples << endl;
    cout << "sampling rate: " << sampling_rate << endl;
    try
    {
        // referencia eeg por canal
        if (do_reference_eeg)
        {
            for (int i = 1; i <= num_channels; i++)
            {
                // datos EEG
                double *data_chan = data.get_address(i);
                for (int n = 0; n < num_samples; n++)
                    // se suman las señales de los canales
                    ref_eeg[n] += data_chan[n];
            }

            // promedio
            for (int n = 0; n < num_samples; n++)
                ref_eeg[n] /= num_channels;
        }
        // else
        // {
        //     for (int i = 1; i <= num_channels; i++)
        //     {
        //         // datos EEG
        //         double *data_chan = data.get_address(i);
        //         for (int n = 0; n < num_samples; n++)
        //         {
        //             // se suman las señales de los canales
        //             ref_eeg[n] += data_chan[n];
        //             // promedio
        //             if (i == num_channels - 1)
        //                 ref_eeg[n] /= num_channels;
        //         }
        //     }
        // }

        // preprocesamiento por canal
        for (int i = 1; i <= num_channels; i++)
        {
            // datos EEG
            double *data_chan = data.get_address(i);

            // PREPROCESAMIENTO
            if (do_reference_eeg || do_reference_eeg_common)
            {
                for (int n = 0; n < num_samples; n++)
                    data_chan[n] -= ref_eeg[n];
            }

            if (do_detrend)
                DataFilter::detrend(data_chan, num_samples, (int)DetrendOperations::LINEAR);

            if (do_erase_noise)
                DataFilter::remove_environmental_noise(data_chan, num_samples, sampling_rate, (int)NoiseTypes::SIXTY);

            if (do_bandpass)
                DataFilter::perform_bandpass(data_chan, num_samples, sampling_rate, freq_ini, freq_end, filter_order, (int)FilterTypes::BUTTERWORTH, 0);
        }

        // guardado de las señales preprocesadas
        if (is_save)
        {
            cout << "Guardando ... " << id_eeg_signal << "_preprocessing.csv" << endl;
            DataFilter::write_file(data, dir_ind + "/" + id_eeg_signal + "_preprocessing.csv", "w");
        }

        // Envío de datos por el canal correspondiente
        eeg_signal_acquisition::eeg_block param;
        param.user_name = user_name;
        param.id_pareto_front = id_pareto_front;
        param.num_ind = num_ind;
        param.id_eeg_signal = id_eeg_signal;
        param.storage_pos = storage_pos;

        param.labels = labels;

        param.time_capturing = time_capturing;
        param.sampling_rate = sampling_rate;
        param.is_start = is_start;
        param.is_stop = is_stop;
        param.type_signal = type_signal;

        ros::NodeHandle n;
        size_t is_controller = type_signal.find("eeg_controller");
        if (is_controller != std::string::npos)
        {
            cout << "send to: preprocessing_eeg_signals" << endl;
            topic_prepro = n.advertise<eeg_signal_acquisition::eeg_block>("preprocessing_eeg_signals", 10);
            topic_prepro.publish(param);
        }
        if (type_signal == "train")
        {
            cout << "send to: preprocessing_eeg_signals_train" << endl;
            topic_prepro_train = n.advertise<eeg_signal_acquisition::eeg_block>("preprocessing_eeg_signals_train", 10);
            topic_prepro_train.publish(param);
        }
        std::size_t is_gui = type_signal.find("gui");
        if (is_gui != std::string::npos)
        {
            vector<double> aux;
            // se pasa a 1d
            for (int i = 1; i <= num_channels; i++)
            {
                double *data_chan = data.get_address(i);
                for (int n = 0; n < num_samples; n++)
                    aux.push_back(data_chan[n]);
            }

            param.eeg_data = aux;
            param.num_channels = num_channels;
            param.num_samples = num_samples;
            cout << "send to: GUI" << endl;
            topic_gui = n.advertise<eeg_signal_acquisition::eeg_block>("gui_eeg_signals", 10);
            topic_gui.publish(param);
        }

        ros::spinOnce();

        cout << "preprocessing end" << endl;
    }
    catch (std::exception &e)
    {
        cout << e.what() << endl;
    }

    return vector<double>();

    cout << "+++++++ end peprocessing +++++++" << endl;
}

void preprocessing_(const eeg_signal_acquisition::eeg_block &msg)
{
    preprocessing_data(msg.labels, msg.eeg_data, msg.num_channels, msg.num_samples, msg.sampling_rate, msg.user_name, msg.id_pareto_front, msg.num_ind, msg.id_eeg_signal, msg.storage_pos, msg.is_start, msg.is_stop, msg.type_signal, msg.time_capturing);
}

bool preprocessing_srv(eeg_signal_acquisition::eeg_block_srv::Request &req,
                       eeg_signal_acquisition::eeg_block_srv::Response &res)
{
    res.eeg_data = preprocessing_data(req.labels, req.eeg_data, req.num_channels, req.num_samples, req.sampling_rate, req.user_name, req.id_pareto_front, req.num_ind, req.id_eeg_signal, req.storage_pos, req.is_start, req.is_stop, req.type_signal, req.time_capturing);
    if (res.eeg_data.size() > 0)
    {
        res.num_channels = req.num_channels;
        res.num_samples = (int)(res.eeg_data.size() / req.num_channels);
    }

    return true;
}

bool get_preprocessing(eeg_signal_acquisition::get_preprocessing_values::Request &req,
                       eeg_signal_acquisition::get_preprocessing_values::Response &res)
{

    res.frequency_ini = freq_ini;
    res.frequency_end = freq_end;
    res.filter_order = filter_order;
    res.remove_environment_noise = do_erase_noise;
    res.perform_bandpass = do_bandpass;
    res.detrend = do_detrend;
    res.is_save = is_save;
    res.reference_eeg = do_reference_eeg;
    res.reference_eeg_common = do_reference_eeg_common;

    return true;
}

bool set_preprocessing(eeg_signal_acquisition::set_preprocessing_values::Request &req,
                       eeg_signal_acquisition::set_preprocessing_values::Response &res)
{

    if (req.frequency_ini > 0)
        freq_ini = req.frequency_ini;
    if (req.frequency_end > 0)
        freq_end = req.frequency_end;
    if (req.filter_order > 0)
        filter_order = req.filter_order;

    do_erase_noise = req.remove_environment_noise;
    do_bandpass = req.perform_bandpass;
    do_detrend = req.detrend;
    is_save = req.is_save;
    do_reference_eeg = req.reference_eeg;
    do_reference_eeg_common = req.reference_eeg_common;

    return true;
}

int main(int argc, char **argv)
{
    // inicio de ros
    ros::init(argc, argv, "preprocessing_eeg");
    ros::NodeHandle n;
    // suscripciones
    ros::Subscriber sub = n.subscribe("eeg_signals", 2000, preprocessing_);

    // servicios
    ros::ServiceServer srv_get_preprocessing = n.advertiseService("get_preprocessing_values", get_preprocessing);
    ros::ServiceServer srv_set_preprocessing = n.advertiseService("set_preprocessing_values", set_preprocessing);
    ros::ServiceServer srv_prepro_signal = n.advertiseService("preprocessing_signal", preprocessing_srv);

    // topicos
    topic_prepro = n.advertise<eeg_signal_acquisition::eeg_block>("preprocessing_eeg_signals", 1000);
    topic_prepro_train = n.advertise<eeg_signal_acquisition::eeg_block>("preprocessing_eeg_signals_train", 1000);
    topic_gui = n.advertise<eeg_signal_acquisition::eeg_block>("gui_eeg_signals", 1000);

    cout << "Nodo preprocessing iniciado con éxito" << endl;
    ros::spin();
    return 0;
}