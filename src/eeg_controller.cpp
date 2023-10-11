#include "ros/ros.h"
#include "eeg_signal_acquisition/eeg_block.h"
#include "eeg_signal_acquisition/control_acquisition.h"
#include "eeg_signal_acquisition/stop_acquisition_eeg.h"
#include "eeg_signal_acquisition/start_acquisition_eeg.h"
#include "eeg_signal_acquisition/get_state_acquisition.h"
#include "eeg_signal_acquisition/connect_board.h"
#include "eeg_signal_acquisition/disconnect_board.h"
#include "eeg_signal_acquisition/change_experiment.h"
#include "eeg_signal_acquisition/get_preprocessing_values.h"
#include "eeg_signal_acquisition/set_preprocessing_values.h"
#include "neurocontroller_database/get_type_id.h"
#include "experiment_control_interfaces/change_state_eeg.h"
#include <vector>
#include <stdlib.h>
#include <unistd.h>
#include <typeinfo>

#include <pwd.h>
#include <thread>
#include <pthread.h>
#include <mutex>

#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <boost/filesystem.hpp>

#include "board_shim.h"
// #include "/home/diego/brainflow/inc/data_filter.h"
#include "data_filter.h"

using namespace std;
struct stat info;

// candados usados para controlar la sincronización
// del hilo que captura las señales EEG
std::mutex mtx;

// contabiliza el tamaño de la señal EEG, desde el inicio
// del flujo hasta que se detiene
int size_experiment = 0;
// señal de paro del hilo que captura las señales EEG
bool stop = false;
// indica si hay una tarjeta conectada
bool connected = false;
// tarjeta
BoardShim *board;
int board_id = 100;
// topico donde se envían las señales EEG
ros::Publisher pub_signals;
// banderas de control en el preprocesamiento
bool do_bandpass = false;
bool do_erase_noise = false;
// rango de frecuencias del filtro
int freq_ini = 4;
int freq_end = 50;
// orden del filtro
int filter_order = 4;
// indica si se publican las señales EEG en un topico ROS
bool is_publish;
// indica si se guardan las señales EEG en un archivo
bool is_save_file;
// indica si se están capturando señales EEG
bool capturing_signals = false;
// cuenta los bloques de señales EEG obtenidos desde el casco
int n_sample = 0;
// tiempo de espera para llenar el buffer de la captura de señales EEG
int time_capturing = 1;
// tamaño de la vista
int window_view = 125 * 5;
int count_window_view = 0;

string id_eeg_signal = "";
string user_name = "";
string id_pareto_front = "";
int num_ind = -1;

pthread_t thread1;

string get_homedir()
{
    struct passwd *pw = getpwuid(getuid());
    string dir(pw->pw_dir);
    return dir;
}

// RUTA RAIZ
std::string root = get_homedir() + "/catkin_ws/src/neurocontroller_database/database";

bool create_dir(string path)
{
    // se verifica si existe el directorio
    bool make = false;
    if (stat(path.c_str(), &info) != 0)
    {
        make = true;
    }
    else
    {
        if (info.st_mode & S_IFDIR)
        {
            return true;
        }
        else
        {
            make = true;
        }
    }

    if (make)
    {
        if (boost::filesystem::create_directories(path))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}

inline bool is_file(const std::string &name)
{
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);

    // bool exist = true;
    // cout << "---1";
    // if( stat( path.c_str(), &info ) != 0 ){
    //     exist = false;
    // }else{
    //     cout << "---2";
    //     if( info.st_mode & S_IFDIR ){// S_ISDIR() doesn't exist on my windows
    //         exist = true;
    //     }else{
    //         exist = false;
    //     }
    // }
    // return exist;
}

bool disconnect_board(eeg_signal_acquisition::disconnect_board::Request &req,
                      eeg_signal_acquisition::disconnect_board::Response &res)
{
    cout << "disconn: " << board << endl;
    try
    {
        if (board->is_prepared())
        {
            board->release_session();
            delete board;
        }
        return true;
    }
    catch (const BrainFlowException &err)
    {
        BoardShim::log_message((int)LogLevels::LEVEL_ERROR, err.what());

        cerr << "Error al desconectar la tarjeta\n";
        return false;
    }
}

void print_vec(std::vector<int> vect)
{
    for (int i = 0; i < vect.size(); i++)
    {
        cout << vect[i] << "  " << endl;
    }
}

bool connect_board(eeg_signal_acquisition::connect_board::Request &req,
                   eeg_signal_acquisition::connect_board::Response &res)
{
    cout << "\n\n------- intentando conexion: " << req.port_path << " -------" << endl;
    cout << "board " << board << endl;
    // if (board->is_prepared ()){
    //     board->release_session ();
    //     delete board;
    // }
    board_id = 100;

    BoardShim::enable_dev_board_logger();
    struct BrainFlowInputParams params;

    if (req.port_path == "simulacion")
    {
        board_id = (int)BoardIds::SYNTHETIC_BOARD;
    }
    else
    {
        board_id = 2; // id de la tarjeta cyton con daisy, ver documentación
    }

    params.serial_port = req.port_path;

    // conexíon con la tarjeta
    try
    {
        board = new BoardShim(board_id, params);
        json board_descr = BoardShim::get_board_descr(board_id);
        int sampling_rate = (int)board_descr["sampling_rate"];
        board->prepare_session();

        cout << "eeg_channels:" << board_descr["eeg_channels"] << endl;
        cout << "eeg_names:" << board_descr["eeg_names"] << endl;
        cout << "name:" << board_descr["name"] << endl;
        cout << "sampling_rate:" << board_descr["sampling_rate"] << endl;
        cout << "battery_channel:" << board_descr["battery_channel"] << endl;
        cout << "accel_channels:" << board_descr["accel_channels"] << endl;
        cout << "gyro_channels:" << board_descr["gyro_channels"] << endl;
        cout << "num_rows:" << board_descr["num_rows"] << endl;
        cout << "rows " << BoardShim::get_num_rows(board_id) << endl;

        // std::vector<int> e2 = BoardShim::get_ecg_channels (board_id);
        // cout << "get_ecg_channels\n";
        // print_vec(e2);

        // std::vector<int> e3 = BoardShim::get_emg_channels (board_id);
        // cout << "get_emg_channels\n";
        // print_vec(e3);

        // std::vector<int> e5 = BoardShim::get_eog_channels (board_id);
        // cout << "get_eog_channels\n";
        // print_vec(e5);

        // std::vector<int> e6 = BoardShim::get_exg_channels (board_id);
        // cout << "get_exg_channels\n";
        // print_vec(e6);

        cout << "------- Tarjeta creada con éxito -------\n";
        connected = true;
        res.msg = "Tarjeta creada con éxito";
        return true;
    }
    catch (...)
    {
        cout << "------- Error al conectar la tarjeta -------\n";
        connected = false;
        res.msg = "Error al conectar la tarjeta";
    }
    return false;
}

void change_state_capturing(bool is_capturing)
{
    ros::NodeHandle n;
    ros::ServiceClient sc = n.serviceClient<experiment_control_interfaces::change_state_eeg>("experiment_control_interfaces/change_state_eeg");

    // parámetros
    experiment_control_interfaces::change_state_eeg param;
    param.request.is_capturing = is_capturing;

    sc.call(param);

    // if(!sc.call(param)){
    //     cerr << "\nError al cambiar el estado de la captura EEG en la interfaz gráfica\n\n";
    // }
}

string get_type_acquisition(string id)
{
    ros::NodeHandle n;
    ros::ServiceClient c = n.serviceClient<neurocontroller_database::get_type_id>("get_type_id");

    neurocontroller_database::get_type_id p_id;
    p_id.request.id_pareto_front = id;

    if (c.call(p_id))
    {
        cout << "type: " << p_id.response.type << endl;
        return p_id.response.type;
    }
    else
    {
        ROS_ERROR("Failed to call service add_two_ints");
        return "unknown";
    }
}

string get_id(string usr, string id_pareto, int n_ind)
{
    // archivo donde se guardan las señales
    string dir = root + "/" + usr + "/" + get_type_acquisition(id_pareto) + "/" + id_pareto + "/ind_" + to_string(n_ind);
    if (!create_dir(dir))
    {
        cout << "Error: no se pudo crear el directorio de salida de las señales EEG\n";
        pthread_exit(NULL);
    }

    // se buscan los archivos de señales  EEG y se extrae el número mayor
    int max = 0;
    for (boost::filesystem::directory_entry &entry : boost::filesystem::directory_iterator(dir))
    {
        string directory = entry.path().string();
        std::size_t found = directory.find("simulation_");
        if (found != std::string::npos)
        {
            string str_num = directory.substr(found + 11);
            std::size_t f = str_num.find(".csv");
            if (f != std::string::npos)
            {
                int n = std::stoi(str_num.substr(0, f));
                if (max < n)
                    max = n;
            }
        }
    }

    return "simulation_" + to_string(max + 1);
}

void *init_acquisition(void *p)
{
    // id de las señales EEG
    id_eeg_signal = get_id(user_name, id_pareto_front, num_ind);
    // tamaño del experimento
    size_experiment = 0;

    ros::Rate loop_rate(1);

    try
    {
        // se prepara la sesión con el casco
        //  board->prepare_session ();
        board->start_stream();

        // buffer de las señales EEG
        vector<vector<double>> signal_eeg(16, vector<double>(0, 0));

        while (ros::ok())
        {
            sleep(time_capturing);

            std::vector<int> eeg_channels = BoardShim::get_eeg_channels(board_id);

            // se obtienen los datos
            BrainFlowArray<double, 2> data = board->get_board_data();

            if (data.get_size(1) == 0)
                continue;

            cout << "mensaje " << n_sample << "\n";
            cout << "data " << data.get_size(1) << " win" << signal_eeg[0].size() << endl;

            // se obtienen la tasa de muestreo
            int sampling_rate = BoardShim::get_sampling_rate(board_id);

            for (int i = 0; i < eeg_channels.size(); i++)
            {
                // se obtienen los datos del casco
                double *aux_vec = data.get_address(eeg_channels[i]);
                // vector<double> data_eeg(aux_vec, aux_vec + data.get_size(1));

                // se concatena la info en el buffer
                for (int k = 0; k < data.get_size(1); k++)
                    signal_eeg[i].push_back(aux_vec[k]);

                // PREPROCESAMIENTO
                if (do_erase_noise)
                {
                    DataFilter::remove_environmental_noise(&signal_eeg[i][0],
                                                           signal_eeg[i].size(), sampling_rate, (int)NoiseTypes::SIXTY);
                }
                if (do_bandpass)
                {
                    DataFilter::perform_bandpass(&signal_eeg[i][0], signal_eeg[i].size(),
                                                 sampling_rate, freq_ini, freq_end, filter_order, (int)FilterTypes::BUTTERWORTH, 0);
                }
            }

            size_experiment += data.get_size(1);

            // se envian las señales en su topico ros
            if (is_publish)
            {
                vector<double> eeg_data;
                // si los datos se van a publicar se guardan en un array unidimensional
                for (int i = 0; i < eeg_channels.size(); i++)
                {
                    eeg_data.insert(eeg_data.end(), signal_eeg[i].begin(), signal_eeg[i].end());
                }

                eeg_signal_acquisition::eeg_block eeg_b;
                eeg_b.eeg_data = eeg_data;
                eeg_b.num_samples = signal_eeg[0].size();
                eeg_b.num_channels = eeg_channels.size();

                eeg_b.id_pareto_front = id_pareto_front;
                eeg_b.num_ind = num_ind;
                eeg_b.id_eeg_signal = id_eeg_signal;
                eeg_b.user_name = user_name;

                eeg_b.time_capturing = time_capturing;
                eeg_b.id = count_window_view;

                if (signal_eeg[0].size() >= window_view)
                {
                    eeg_b.creating_signal = false;
                    count_window_view++;
                    vector<vector<double>> aux_sig(16, vector<double>(0, 0));
                    signal_eeg = aux_sig;
                }
                else
                {
                    eeg_b.creating_signal = true;
                }

                // se publican las señales
                pub_signals.publish(eeg_b);
                // ros::spinOnce();
            }

            // se guardan las señales en un archivo
            if (is_save_file)
            {
                string dir_exp = root + "/" + user_name + "/" + get_type_acquisition(id_pareto_front) + "/" + id_pareto_front + "/ind_" + to_string(num_ind);
                string dir_file = dir_exp + "/" + id_eeg_signal + ".csv";

                cout << "wriet 1: " << data.get_size(0) << " 2: " << data.get_size(1) << endl;
                DataFilter::write_file(data, dir_file, "a");

                BrainFlowArray<double, 2> read_data = DataFilter::read_file(dir_file);
                cout << "read 1: " << read_data.get_size(0) << " 2: " << read_data.get_size(1) << endl;
            }

            // se detiene la captura de las señales EEG
            if (stop)
            {
                cout << "Deteniendo la adquisición de señales EEG: " << n_sample << "\n";
                eeg_signal_acquisition::eeg_block eeg_b;
                eeg_b.num_channels = eeg_channels.size();
                eeg_b.id_pareto_front = id_pareto_front;
                eeg_b.num_ind = num_ind;
                eeg_b.id_eeg_signal = id_eeg_signal;
                eeg_b.user_name = user_name;
                eeg_b.time_capturing = time_capturing;
                eeg_b.is_stop = true;
                pub_signals.publish(eeg_b);

                // eeg_signal_acquisition::eeg_block eeg_b;
                // eeg_b.is_stop = true;
                // //se publican las señales
                // pub_signals.publish(eeg_b);

                // deteniendo flujo EEG
                board->stop_stream();
                // board->release_session ();

                // se cambia el estado de captura
                change_state_capturing(false);

                mtx.lock();
                // indicador que esta detenido el flujo EEG
                stop = false;
                // indicador que ya no hay hilos ejecutando la captura
                capturing_signals = false;
                mtx.unlock();

                break;
            }

            // actualización del estado de la interfaz
            if (n_sample % 10 == 0)
            {
                // se cambia el estado de captura
                change_state_capturing(true);
            }

            // loop_rate.sleep();
            n_sample++;
        }
    }
    catch (const BrainFlowException &err)
    {
        BoardShim::log_message((int)LogLevels::LEVEL_ERROR, err.what());
    }
}

bool change_exper(eeg_signal_acquisition::change_experiment::Request &req,
                  eeg_signal_acquisition::change_experiment::Response &res)
{
    user_name = req.user_name;
    id_pareto_front = req.id_pareto_front;
    num_ind = req.num_ind;

    id_eeg_signal = get_id(user_name, id_pareto_front, num_ind);

    return true;
}

bool control(eeg_signal_acquisition::control_acquisition::Request &req,
             eeg_signal_acquisition::control_acquisition::Response &res)
{

    if (req.sampling_rate)
    {
        int sampling = BoardShim::get_sampling_rate(board_id);
        cout << "\n\nGet sampling rate:" << sampling << endl;
        res.sampling_rate = sampling;
        return true;
    }
}

bool stop_acquisition(eeg_signal_acquisition::stop_acquisition_eeg::Request &req,
                      eeg_signal_acquisition::stop_acquisition_eeg::Response &res)
{
    cout << "\n\n ----- Deteniendo el flujo de señales EEG -----\n";
    // señal de paro
    stop = true;

    res.id_eeg_signal = id_eeg_signal;
    res.size_experiment = size_experiment;

    // esperamos a que termine el hilo
    pthread_join(thread1, NULL);

    cout << "USER_NAME: " << user_name << endl;
    cout << "ID_PARETO_FRONT: " << id_pareto_front << endl;
    cout << "NUM_IND: " << num_ind << endl;
    cout << "ID_EEG_SIGNAL: " << id_eeg_signal << endl;
    cout << "size_experiment: " << size_experiment << endl;
    cout << "------ flujo de señales EEG detenidas ---------\n";

    return true;
}

bool start_acquisition(eeg_signal_acquisition::start_acquisition_eeg::Request &req,
                       eeg_signal_acquisition::start_acquisition_eeg::Response &res)
{
    cout << "\n\n----- Iniciando el flujo de señales EEG -----\n";
    mtx.lock();

    // sin conexión con la tarjeta
    if (board_id == 100)
    {
        cout << "+++++++++ No hay conexión con la tarjeta ++++++++++\n";
        mtx.unlock();
        return false;
    }

    // se inicia el hilo que captura las señales EEG
    if (!capturing_signals)
    {
        capturing_signals = true;
        stop = false;
        mtx.unlock();

        user_name = req.user_name;
        id_pareto_front = req.id_pareto_front;
        num_ind = req.num_ind;
        is_publish = req.is_publish;
        is_save_file = req.is_save_file;

        // se inicia el hilo de captura EEG
        int iret = pthread_create(&thread1, NULL, init_acquisition, NULL);
    }
    else
        mtx.unlock();

    res.id_eeg_signal = id_eeg_signal;
    // cout << "----- Experimento terminado, ID_EEG_SIGNAL: " << id_eeg_signal << " --------\n";
    return true;
}

bool get_state_acquisition(eeg_signal_acquisition::get_state_acquisition::Request &req,
                           eeg_signal_acquisition::get_state_acquisition::Response &res)
{
    // cout << "---- get acquisition metadata ----- "<<endl;

    if (capturing_signals)
        res.state = "start";
    else
        res.state = "stop";

    res.id_eeg_signal = id_eeg_signal;
    res.num_ind = num_ind;
    res.connected = connected;

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

    return true;
}

int main(int argc, char **argv)
{

    // inicio de ros
    ros::init(argc, argv, "eeg_headset");
    ros::NodeHandle n;
    // donde se reciben los mensajes de control
    ros::ServiceServer srv_ctl = n.advertiseService("control_acquisition", control);
    ros::ServiceServer srv_start = n.advertiseService("start_acquisition_eeg", start_acquisition);
    ros::ServiceServer srv_stop = n.advertiseService("stop_acquisition_eeg", stop_acquisition);
    // carga las señales eeg
    ros::ServiceServer srv_connect = n.advertiseService("connect_board", connect_board);
    ros::ServiceServer srv_disconnect = n.advertiseService("disconnect_board", disconnect_board);

    ros::ServiceServer srv_get_preprocessing = n.advertiseService("get_preprocessing_values", get_preprocessing);
    ros::ServiceServer srv_set_preprocessing = n.advertiseService("set_preprocessing_values", set_preprocessing);

    // consulta del estado
    ros::ServiceServer srv_state = n.advertiseService("get_state_acquisition", get_state_acquisition);
    // cambia los valores de donde se guarda la info de las señales EEG
    ros::ServiceServer srv_change = n.advertiseService("change_experiment", change_exper);

    pub_signals = n.advertise<eeg_signal_acquisition::eeg_block>("eeg_signals", 500);

    ros::spin();
    return 0;
}
