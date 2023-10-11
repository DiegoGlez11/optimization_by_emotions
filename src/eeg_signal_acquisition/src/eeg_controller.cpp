// ros imports
#include "ros/ros.h"
#include "eeg_signal_acquisition/eeg_block.h"
#include "eeg_signal_acquisition/control_acquisition.h"
#include "eeg_signal_acquisition/stop_acquisition_eeg.h"
#include "eeg_signal_acquisition/start_acquisition_eeg.h"
#include "eeg_signal_acquisition/get_state_acquisition.h"
#include "eeg_signal_acquisition/connect_board.h"
#include "eeg_signal_acquisition/disconnect_board.h"
// #include "eeg_signal_acquisition/change_experiment.h"
#include "neurocontroller_database/get_type_id.h"
#include "neurocontroller_database/get_id.h"
#include "experiment_control_interfaces/change_state_eeg.h"
#include "experiment_control_interfaces/change_state_eeg_msg.h"

#include <vector>
#include <stdlib.h>
#include <unistd.h>
#include <typeinfo>
#include <string>
#include <pwd.h>
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <boost/filesystem.hpp>

// para la sincronización
#include <thread>
#include <pthread.h>
#include <mutex>

// tiempo
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::system_clock;

// brainflow
#include "board_shim.h"
#include "data_filter.h"

// duerme del programa en milisegundos
#define SLEEP_MS(milsec) usleep(milsec * 1000)

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
// id de la tarjeta
int board_id = 100;
// descripción de la tarjet
json board_descr;
// topico donde se envían las señales EEG
ros::Publisher pub_signals;
// ros::Publisher pub_signals_prepro;
ros::Publisher pub_state;

// indica si se publican las señales EEG en un topico ROS
bool is_publish;
// indica si se guardan las señales EEG en un archivo
bool is_save_file;
// si se va a preprocesar las señales
//  bool is_preprocessing = true;
//  indica si se están capturando señales EEG
bool capturing_signals = false;

// tiempo de espera para llenar el buffer de la captura de señales EEG
double time_capturing;
// tamaño de la vista
int window_view = 125 * 5;

int sampling_rate;

string id_eeg_signal = "";
string user_name = "";
string id_pareto_front = "";
int num_ind = -1;
int storage_pos = -1;

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

// void change_state_capturing(bool is_capturing)
// {
//     ros::NodeHandle n;

//     // parámetros
//     experiment_control_interfaces::change_state_eeg param;
//     param.request.is_capturing = is_capturing;
//     param.request.is_connected = connected;

//     ros::ServiceClient sc = n.serviceClient<experiment_control_interfaces::change_state_eeg>("experiment_control_interfaces/change_state_eeg");
//     // sc.call(param);

//     if (sc.waitForExistence(ros::Duration(0, 100)))
//     {
//         cout << "exist service and valid" << sc.isValid() << endl;
//         if (!sc.call(param))
//         {
//             cerr << "\nError al cambiar el estado de la captura EEG en la interfaz gráfica\n\n";
//         }
//     }
//     // if (ros::service::exists("experiment_control_interfaces/change_state_eeg", true))
//     // {
//     //     if (!ros::service::call("experiment_control_interfaces/change_state_eeg", param))
//     //     {
//     //         cerr << "\nError al cambiar el estado de la captura EEG en la interfaz gráfica\n\n";
//     //     }
//     // }

//     cout << "change state of connection" << endl;
// }

void change_state_capturing(bool is_capturing)
{
    ros::NodeHandle n;

    // parámetros
    experiment_control_interfaces::change_state_eeg_msg param;
    param.is_capturing = is_capturing;
    param.is_connected = connected;

    ros::Publisher pub_state = n.advertise<experiment_control_interfaces::change_state_eeg_msg>("change_state_eeg_msg", 10);

    pub_state.publish(param);
    ros::spinOnce();

    cout << "change state of connection" << endl;
}

bool disconnect_board()
{
    cout << "++++++ desconectando tarjeta +++++++: " << board_id << endl;
    if (board_id == 100)
        return true;

    try
    {
        if (board->is_prepared())
        {
            // deteniendo flujo EEG. Desde que se conecta el casco EEG se inicia
            // el flujo EEG
            board->stop_stream();
            // se libera la sesión
            board->release_session();

            // delete board;
        }
        // se elimina la tarjeta
        delete board;
        // se resetea el id de la tarjeta
        board_id = 100;
        // no hay conexión
        connected = false;
        // cambio en la GUI
        change_state_capturing(false);
    }
    catch (const BrainFlowException &err)
    {
        BoardShim::log_message((int)LogLevels::LEVEL_ERROR, err.what());
        // se resetea el id de la tarjeta
        board_id = 100;
        // no hay conexión
        connected = false;
        // cambio en la GUI
        change_state_capturing(false);

        cerr << "Error al desconectar la tarjeta\n";
        return false;
    }
    return true;
}

bool disconnect_board_srv(eeg_signal_acquisition::disconnect_board::Request &req,
                          eeg_signal_acquisition::disconnect_board::Response &res)
{
    bool res_dis = disconnect_board();
    return res_dis;
}

bool connect_board(eeg_signal_acquisition::connect_board::Request &req,
                   eeg_signal_acquisition::connect_board::Response &res)
{
    cout << "\n\n------- intentando conexion: " << req.port_path << " -------" << endl;
    // selección del id de la tarjeta
    int board_id_aux = 100;
    if (req.port_path == "simulacion")
        board_id_aux = (int)BoardIds::SYNTHETIC_BOARD;
    else
        board_id_aux = 2; // id de la tarjeta cyton con daisy, ver documentación

    // se verifica si ya esta conectada la tarjeta
    cout << connected << endl;
    if (connected)
    {
        // mismo ID
        if (board_id_aux == board_id)
        {
            cout << "------- La tarjeta ya se encuentra conectada -------\n";
            cout << "eeg_channels:" << board_descr["eeg_channels"] << endl;
            cout << "eeg_names:" << board_descr["eeg_names"] << endl;
            cout << "name:" << board_descr["name"] << endl;
            cout << "sampling_rate:" << board_descr["sampling_rate"] << endl;
            cout << "battery_channel:" << board_descr["battery_channel"] << endl;
            cout << "accel_channels:" << board_descr["accel_channels"] << endl;
            cout << "gyro_channels:" << board_descr["gyro_channels"] << endl;
            cout << "num_rows:" << board_descr["num_rows"] << endl;
            cout << "rows " << BoardShim::get_num_rows(board_id) << endl;

            return true;
        }
        else // diferente ID = nueva tarjeta
        {
            // se desconecta
            disconnect_board();
        }
    }
    // ID de la tarjeta a conectar
    board_id = board_id_aux;
    if (board_id == 100)
    {
        cerr << "----- Tarjeta inválida ----" << endl;
        return false;
    }

    BoardShim::enable_dev_board_logger();
    struct BrainFlowInputParams params;

    params.serial_port = req.port_path;

    // conexíon con la tarjeta
    try
    {
        board = new BoardShim(board_id, params);
        board_descr = BoardShim::get_board_descr(board_id);
        sampling_rate = (int)board_descr["sampling_rate"];
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

        cout << "------- Tarjeta creada con éxito -------\n";
        connected = true;

        // se inicia el flujo. Se coloca en éste lugar para evitar estar
        // iniciando y cerrando el flujo EEG, es más carga para el dispositivo
        // y se evitan las señales ondulatorias que se producen  al incio después de
        // haber comenzado el flujo EEG
        board->start_stream();

        res.msg = "Tarjeta creada con éxito";
        return true;
    }
    catch (const BrainFlowException &err)
    {
        // board->prepare_session();
        BoardShim::log_message((int)LogLevels::LEVEL_ERROR, err.what());
        cout << "------- Error al conectar la tarjeta -------" << endl;

        connected = false;
        res.msg = "Error al conectar la tarjeta";
    }
    return false;
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
        ROS_ERROR("Error al obtener el tipo de ID");
        return "unknown";
    }
}

string get_id(string usr, string id_pareto, int n_ind)
{
    // archivo donde se guardan las señales
    string dir = root + "/" + usr;

    // cuando es estado basal
    std::size_t f_b = id_pareto.find("optimized_basal");
    if (f_b != std::string::npos)
        dir = dir + "/" + id_pareto;
    else
        dir = dir + "/" + get_type_acquisition(id_pareto) + "/" + id_pareto + "/ind_" + to_string(n_ind);

    if (!create_dir(dir))
    {
        cout << "Error: no se pudo crear el directorio de salida de las señales EEG\n";
        pthread_exit(NULL);
    }

    string type_board = "unknown";
    if (board_id == -1)
        type_board = "synthetic";
    if (board_id == 2)
        type_board = "cytonDaisy";

    string id_estimuli = "estimulation_" + type_board + "-";

    ros::NodeHandle n;
    ros::ServiceClient srv_get_id = n.serviceClient<neurocontroller_database::get_id>("get_id");

    neurocontroller_database::get_id param_id;
    param_id.request.key = id_estimuli;
    param_id.request.directory = dir;

    if (srv_get_id.call(param_id))
    {
        return param_id.response.id;
    }
    return "";
    // return id_estimuli + "-" + to_string(max + 1);
}

void publish_data(BrainFlowArray<double, 2> data_eeg, bool is_start, bool is_stop)
{
    // vector<double> data_eeg_1d;

    // // si los datos se van a publicar se guardan en un array unidimensional
    std::vector<int> eeg_channels = BoardShim::get_eeg_channels(board_id);
    cout << "channels" << endl;
    for (int i = 0; i < eeg_channels.size(); i++)
    {
        cout << eeg_channels[i] << " ";
        // se obtienen los datos del casco
        // double *aux_vec = data_eeg.get_address(eeg_channels[i]);

        // // se concatena la info en el buffer
        // for (int k = 0; k < data_eeg.get_size(1); k++)
        //     data_eeg_1d.push_back(aux_vec[k]);
    }
    cout << endl;

    eeg_signal_acquisition::eeg_block eeg_b;
    // eeg_b.eeg_data = data_eeg_1d;
    // eeg_b.num_samples = data_eeg.get_size(1);
    // eeg_b.num_channels = eeg_channels.size();

    eeg_b.id_pareto_front = id_pareto_front;
    eeg_b.num_ind = num_ind;
    eeg_b.id_eeg_signal = id_eeg_signal;
    eeg_b.user_name = user_name;
    eeg_b.storage_pos = storage_pos;

    eeg_b.time_capturing = time_capturing;
    eeg_b.is_stop = is_stop;
    eeg_b.is_start = is_start;

    eeg_b.sampling_rate = sampling_rate;
    eeg_b.type_signal = "eeg_controller";

    // se publican las señales
    ros::NodeHandle n;
    pub_signals = n.advertise<eeg_signal_acquisition::eeg_block>("eeg_signals", 20);
    pub_signals.publish(eeg_b);
    ros::spinOnce();

    cout << "publish" << endl;
}

void *init_acquisition(void *p)
{
    // id de las señales EEG
    id_eeg_signal = get_id(user_name, id_pareto_front, num_ind);
    // tamaño del experimento
    size_experiment = 0;

    // dirección del archivo de las señales EEG
    string dir_file = root + "/" + user_name;

    // dir estado basal
    std::size_t f_b = id_pareto_front.find("optimized_basal");
    if (f_b != std::string::npos)
        dir_file = dir_file + "/" + id_pareto_front;
    else
        dir_file = dir_file + "/" + get_type_acquisition(id_pareto_front) + "/" + id_pareto_front + "/ind_" + to_string(num_ind);

    cout << dir_file << endl;

    // cuenta los bloques de señales EEG obtenidos desde el casco
    int n_sample = 0;

    ros::Rate loop_rate(1);

    // se cambia el estado de captura
    change_state_capturing(true);

    try
    {
        // se obtiene la tasa de muestreo
        sampling_rate = BoardShim::get_sampling_rate(board_id);
        BrainFlowArray<double, 2> data_eeg;

        // se inicia el flujo
        // board->start_stream();
        // tiempo
        time_t end_time, ini_time;
        while (ros::ok())
        {
            ini_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

            // vaciado del buffer, se hace porque se mantiene abierta la adquisicion.
            // Aunque se detenga la función init_acquisition() el buffer se sigue llenando
            // si se desea detener la adquisicion usar board->stop_stream()
            board->get_board_data();

            // el buffer se debe llenar hasta el tamaño buscado
            // while (board->get_board_data_count() < window_view)
            while (!stop)
            {
                SLEEP_MS(8);
            }
            end_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
            time_capturing = end_time - ini_time;

            cout << "tiempo de adquisición: " << time_capturing << "ms" << endl;

            // se obtienen los datos EEG
            data_eeg = board->get_board_data();
            size_experiment += data_eeg.get_size(1);

            cout << "mensaje " << n_sample << endl;
            cout << "data " << data_eeg.get_size(1) << endl;

            // se guardan las señales en un archivo
            if (is_save_file)
            {
                DataFilter::write_file(data_eeg, dir_file + "/" + id_eeg_signal + ".csv", "w");
            }

            // se detiene la captura de las señales EEG
            if (stop)
            {
                cout << "Deteniendo la adquisición de señales EEG: " << n_sample << "\n";

                if (is_publish)
                {
                    publish_data(data_eeg, false, true);
                }

                // deteniendo flujo EEG
                // board->stop_stream();
                // board->release_session ();

                mtx.lock();
                // indicador que esta detenido el flujo EEG
                stop = false;
                // indicador que ya no hay hilos ejecutando la captura
                capturing_signals = false;
                mtx.unlock();

                break;
            }
            // else
            // {
            //     // se envian las señales en su topico ros
            //     if (is_publish)
            //     {
            //         if (n_sample == 0)
            //             publish_data(data_eeg, true, false);
            //         else
            //             publish_data(data_eeg, false, false);
            //     }
            // }

            // actualización del estado de la interfaz
            if (n_sample % 4 == 0)
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

bool control(eeg_signal_acquisition::control_acquisition::Request &req,
             eeg_signal_acquisition::control_acquisition::Response &res)
{

    if (req.sampling_rate)
    {
        sampling_rate = BoardShim::get_sampling_rate(board_id);
        cout << "\n\nGet sampling rate:" << sampling_rate << endl;
        res.sampling_rate = sampling_rate;
        return true;
    }
    if (req.board_id)
    {
        res.board_id = board_id;
    }

    // is_preprocessing = req.preprocessing;
}

bool stop_acquisition(eeg_signal_acquisition::stop_acquisition_eeg::Request &req,
                      eeg_signal_acquisition::stop_acquisition_eeg::Response &res)
{
    cout << "\n\n ----- Deteniendo el flujo de señales EEG -----\n";
    // señal de paro
    stop = true;

    cout << "end thread " << thread1 << endl;
    // esperamos a que termine el hilo
    pthread_join(thread1, NULL);

    // se cambia el estado de captura
    change_state_capturing(false);

    res.id_eeg_signal = id_eeg_signal;
    res.size_experiment = size_experiment;

    cout << "USER_NAME: " << user_name << endl;
    cout << "ID_PARETO_FRONT: " << id_pareto_front << endl;
    cout << "NUM_IND: " << num_ind << endl;
    cout << "ID_EEG_SIGNAL: " << id_eeg_signal << endl;
    cout << "storage_pos: " << storage_pos << endl;
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
        storage_pos = req.storage_pos;

        is_publish = req.is_publish;
        is_save_file = req.is_save_file;

        // se inicia el hilo de captura EEG
        int iret = pthread_create(&thread1, NULL, init_acquisition, NULL);
        cout << "init thread " << thread1 << endl;
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
    if (capturing_signals)
        res.state = "start";
    else
        res.state = "stop";

    res.id_eeg_signal = id_eeg_signal;
    res.num_ind = num_ind;
    res.connected = connected;

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
    ros::ServiceServer srv_disconnect = n.advertiseService("disconnect_board", disconnect_board_srv);

    // consulta del estado
    ros::ServiceServer srv_state = n.advertiseService("get_state_acquisition", get_state_acquisition);
    // cambia los valores de donde se guarda la info de las señales EEG
    // ros::ServiceServer srv_change = n.advertiseService("change_experiment", change_exper);

    pub_signals = n.advertise<eeg_signal_acquisition::eeg_block>("eeg_signals", 20);
    // pub_signals_prepro = n.advertise<eeg_signal_acquisition::eeg_block>("preprocessing_eeg_signals", 2000);
    pub_state = n.advertise<experiment_control_interfaces::change_state_eeg_msg>("change_state_eeg_msg", 10);

    cout << "Nodo eeg_controller iniciado con éxito" << endl;

    // canbio de estado de la conexión
    change_state_capturing(false);

    ros::spin();
    return 0;
}
