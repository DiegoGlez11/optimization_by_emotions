#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include "ros/ros.h"
#include <cmath>

#include "neurocontroller_database/load_ind.h"
#include "neurocontroller_database/load_phylogenetic.h"
#include "neurocontroller_database/save_phylogenetic.h"
#include "neurocontroller_database/load_obj_space.h"
#include "neurocontroller_database/save_data_string.h"
#include "neurocontroller_database/get_id.h"
#include "neurocontroller_database/get_type_id.h"

#include <vector>
#include <string>

#include <boost/filesystem.hpp>

#include <sys/types.h>
#include <pwd.h>

// #include "my_node/get_neurocontroller_data.h"
// #include "neurocontroller_database/load_pop.h"
// #include "neurocontroller_database/save_pop.h"

using namespace std;
struct stat info;

int num_solicitud = 0;

string get_homedir()
{
    struct passwd *pw = getpwuid(getuid());
    string dir(pw->pw_dir);
    return dir;
}

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
        { // S_ISDIR() doesn't exist on my windows
            // cout << "La población cuenta con su directorio: " << path << endl;
            return true;
        }
        else
        {
            make = true;
        }
    }

    if (make)
    {
        cout << "Creando directorio de la población" << endl;
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

bool is_dir(string path)
{
    bool exist = true;
    if (stat(path.c_str(), &info) != 0)
    {
        exist = false;
    }
    else
    {
        if (info.st_mode & S_IFDIR)
        { // S_ISDIR() doesn't exist on my windows
            exist = true;
        }
        else
        {
            exist = false;
        }
    }
    return exist;
}

// ++++++++++++++++ RUTA RAIZ +++++++++++++++
std::string root = get_homedir() + "/catkin_ws/src/neurocontroller_database/database";

int get_num_weights(std::vector<int> vec)
{
    vector<int> poi;
    // eliminamos los ceros de la arquitectura de la ANN
    for (int i = 0; i < vec.size(); i++)
    {
        if (vec[i] > 0)
            poi.push_back(vec[i]);
    }

    if (poi.size() > 0)
    {
        int sum = 0;
        for (int i = 0; i < poi.size() - 1; i++)
        {
            sum += (poi[i] + 1) * poi[i + 1]; // el mas 1 es por el valor del sesgo
        }
        return sum;
    }
    else
    {
        return 0;
    }
}

vector<double> get_numbers(string txt)
{
    std::vector<double> data;
    int estado = 1;
    std::string str_num = "";
    int i = 0;
    while (i < txt.length())
    {

        // estado automata
        if (estado == 3 && txt[i] >= 48 && txt[i] <= 57)
            estado = 4;
        if (estado == 2 && txt[i] == '.')
            estado = 4;
        if (estado == 1)
        {
            if (txt[i] == '.')
                estado = 3;
            if (txt[i] >= 48 && txt[i] <= 57)
                estado = 2;
            if (txt[i] == '-')
                estado = 2;
        }

        // acciones
        if (estado > 1)
        {
            str_num += txt[i];
            if (estado == 2 || estado == 4)
            {
                if (i < txt.length())
                {
                    if (txt[i + 1] != '-' && txt[i + 1] != '.' && (txt[i + 1] < 48 || txt[i + 1] > 57))
                    {
                        // cout << str_num << endl;
                        data.push_back(std::stof(str_num));
                    }
                }
                else
                {
                    data.push_back(std::stof(str_num));
                    // cout << str_num << endl;
                }
            }
        }
        if (txt[i + 1] != '-' && txt[i + 1] != '.' && (txt[i + 1] < 48 || txt[i + 1] > 57))
        {
            estado = 1;
            str_num = "";
        }
        i++;
    }
    return data;
}

std::vector<double> read_file(string path)
{
    ifstream file(path);
    if (file.fail())
    {
        vector<double> vect;
        return vect;
    }
    std::vector<double> data;
    std::string txt;
    while (getline(file, txt))
    {
        int estado = 1;
        std::string str_num = "";
        int i = 0;
        bool shape = false;
        while (i < txt.length())
        {
            // cout <<i <<":"<< txt[i] << " - " << estado << endl;

            // estado automata
            if (estado == 3 && txt[i] >= 48 && txt[i] <= 57)
                estado = 4;
            if (estado == 2 && txt[i] == '.')
                estado = 4;
            if (estado == 1)
            {
                if (txt[i] == '.')
                    estado = 3;
                if (txt[i] >= 48 && txt[i] <= 57)
                    estado = 2;
                if (txt[i] == '-')
                    estado = 2;
            }

            // acciones
            if (estado > 1)
            {
                str_num += txt[i];
                if (estado == 2 || estado == 4)
                {
                    if (i < txt.length())
                    {
                        if (txt[i + 1] != '-' && txt[i + 1] != '.' && (txt[i + 1] < 48 || txt[i + 1] > 57))
                        {
                            // cout << str_num << endl;
                            data.push_back(std::stof(str_num));
                        }
                    }
                    else
                    {
                        data.push_back(std::stof(str_num));
                        // cout << str_num << endl;
                    }
                }
            }
            if (txt[i] == '#')
                shape = true;
            if (txt[i + 1] != '-' && txt[i + 1] != '.' && (txt[i + 1] < 48 || txt[i + 1] > 57))
            {
                estado = 1;
                str_num = "";
            }
            i++;
        }
    }
    return data;
}

string load_phylo(string user_name, string id_pareto_front)
{
    string dir = root + "/" + user_name + "/pareto_fronts/front" + id_pareto_front + "/optimization";
    // se verifica si existe el directorio
    if (is_dir(dir))
    {
        // abrimos el archivo
        string dir_file = dir + "/phylogenetic.txt";
        std::ifstream file_phylo(dir_file);
        cout << "dirección: " << dir_file << endl;
        std::stringstream buffer;
        buffer << file_phylo.rdbuf();
        return buffer.str();
        // if(file_phylo){
        //     file_phylo.seekg(0, file_phylo.end);
        //     int size = file_phylo.tellg();
        //     file_phylo.seekg(0, file_phylo.beg);

        //     char* data = new char[size];
        //     file_phylo.read(data, size);
        //     string dat(data);
        //     return dat;
        // }else{
        //     return "";
        // }
    }
    else
    {
        return "";
    }
}

bool load_phylogenetic(neurocontroller_database::load_phylogenetic::Request &req, neurocontroller_database::load_phylogenetic::Response &res)
{
    cout << "\n\n------------ Cargando información filogenética -------------   \n";

    string data = "";
    data = load_phylo(req.user_name, req.id_pareto_front);
    res.phylogenetic = data;

    cout << "------------ información filogenetica cargada con éxito-------------\n";
    return true;
}

bool save_data_str(string data_str, string dir, string name)
{
    // is dir
    if (!is_dir(dir))
    {
        if (!create_dir(dir))
            return false;
    }

    // fstream outfile;

    // FILE *o_file = fopen(dir_phylo.c_str(), "w+");
    // if (o_file){
    //     fwrite(phylo.c_str(), 1, phylo.size(), o_file);
    //     return true;
    // }
    string path = dir + "/" + name;

    ofstream f_p(path);
    // f_p.open(, std::ios_base::out);
    if (f_p.fail())
    {
        cout << "Error al guardar en:" << path << endl;
        return false;
    }
    else
    {
        // se guarda la info filogenetica en el archivo
        f_p << data_str;
        f_p.close();
        cout << "++ Se guardó con éxito el string en " << path << " ++\n";
        return true;
    }
    return false;
}

bool save_str(neurocontroller_database::save_data_string::Request &req, neurocontroller_database::save_data_string::Response &res)
{
    cout << "Guardando archivo :" << req.directory << "/" << req.name_file;
    bool saved = save_data_str(req.data_string, req.directory, req.name_file);
    return saved;
}

bool save_phylogenetic(neurocontroller_database::save_phylogenetic::Request &req, neurocontroller_database::save_phylogenetic::Response &res)
{
    string data_phylo = req.phylogenetic;
    string usr = req.user_name;
    string id = req.id_pareto_front;

    cout << "\n\n----- Cargando información filogenética ------\n";
    cout << "user_name: " << usr << " id_pareto:" << id << endl;

    string dir = root + "/" + usr + "/pareto_fronts/front" + id + "/optimization";
    string name = "phylogenetic.txt";

    if (save_data_str(data_phylo, dir, name))
    {
        cout << "----- Guardado de la información filogenetica: " << id << " usuario:" << usr << " -----\n";
        return true;
    }
    else
    {
        string err(strerror(errno));
        cout << "++++++ Error al guardar la información filogenetica: " + err + " ++++++\n";
        return false;
    }
}

inline bool is_file(const std::string &name)
{
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}

double str_to_float(string str_num)
{
    cout << "input --- " << str_num << endl;
    size_t pos = str_num.find(".");
    string str_int = str_num.substr(0, pos);
    string str_dec = str_num.substr(pos + 1);
    double num_int = stod(str_int);

    double signo;
    if (num_int >= 0)
        signo = 1;
    else
        signo = -1;
    double sum = num_int;
    for (int j = 0; j < str_dec.size(); j++)
    {
        double aux = str_dec.at(j) - '0';
        double n_d = aux / pow(10, j + 1);
        sum += signo * n_d;
    }

    // long double num_dec = stod(str_dec)/100000000;
    // cout <<"sep "<< num_int << " - "<<num_dec << endl;
    // long double sum;
    // if(num_int >= 0) sum = num_int + num_dec;
    // else sum = num_int - num_dec;

    return sum;
}

// convierte el ID de un frente optimizado en la ruta de su archivo
//  type: posibles valores = (var_space, obj_space)
string get_path_optimized_ind(string id_pareto, string type)
{
    size_t f = id_pareto.find("optimized");
    string path_file = "";
    // cout << "pos " << f << " id " << id_pareto << endl;

    if (f != std::string::npos)
    {
        // num gen
        string n_gen = id_pareto.substr(f + 10);
        stringstream aux;
        aux << n_gen;
        int num_gen;
        aux >> num_gen;

        if (num_gen < 10)
            n_gen = "00" + n_gen;
        else
            n_gen = "0" + n_gen;

        path_file = root + "_populations/optimized_individuals/" + type + "_gen_" + n_gen + ".out";
        cout << path_file << endl;
        if (is_file(path_file))
        {
            return path_file;
        }
        else
        {
            return "";
        }
    }
    return path_file;
}

vector<double> string_to_values(string str_array)
{
    // pasamos el string a un stream para extraer los números
    stringstream data_stream;
    data_stream << str_array;
    // variable auxiliar para extraer los números
    double num;
    // guarda los valores extraidos del string
    vector<double> data;

    while (true)
    {
        data_stream >> num;
        // posición del stream
        int pos = data_stream.tellg();
        // se verifica que continuemos dentro del stream
        if (pos < 0)
            break;
        else
        {
            data.push_back(num);
            // cout << "num: " << std::setprecision(15) << num << endl;
            // cout << "pos: " << pos << endl;
            // cout << "sub: " << str_array.substr(0,pos) << endl;
        }
    }
    return data;
}

vector<string> get_fronts(string id_pareto)
{

    size_t is_comp = id_pareto.find(",");
    size_t is_base = id_pareto.find("optimized-");

    // no es un frente válido
    if (is_comp == std::string::npos && is_base == std::string::npos)
    {
        cerr << "Frente no válido, no se puede cargar" << endl;
    }

    vector<string> ids_front;
    // si es un frente compuesto
    if (is_comp != std::string::npos)
    {
        int i = 0;
        string delimiter = ",";
        size_t pos = 0;
        std::string token;
        while ((pos = id_pareto.find(delimiter)) != std::string::npos)
        {
            token = id_pareto.substr(0, pos);
            id_pareto.erase(0, pos + delimiter.length());
            if (i == 0)
                ids_front.push_back(token);
            else
                ids_front.push_back("optimized-" + token);
            i++;
        }
        ids_front.push_back("optimized-" + id_pareto);
    }
    else
    {
        ids_front.push_back(id_pareto);
    }

    return ids_front;
}

bool load_obj_space_by_front(neurocontroller_database::load_obj_space::Request &req, neurocontroller_database::load_obj_space::Response &res)
{
    // se sabe apriori el tamaño de objetivos
    int num_obj = 4;
    string id_pareto = req.id_pareto_front;
    cout << "\n\n----------- Cargando espacio de los objetivos ---------------\n";

    // vector<string> ids = get_fronts(id_pareto);
    // for (int i = 0; i < ids.size(); i++)
    // {
    //     // frente a cargar
    //     string id_front = ids[i];
    //     string dir = get_path_optimized_ind(id_front, "obj_space");

    //     // vector de vectores
    //     vector<double> all_fronts;

    //     // abrimos el archivo
    //     string txt;
    //     fstream file_ind;
    //     file_ind.open(dir, std::ios_base::in);
    //     if (file_ind)
    //     {
    //         // leemos todo el contenido del archivo
    //         ostringstream os;
    //         os << file_ind.rdbuf();
    //         txt = os.str();

    //         // convertimos el texto a números
    //         vector<double> values = string_to_values(txt);
    //         cout << "size array: " << values.size() << endl;
    //     }
    // }

    // return true;

    string dir = get_path_optimized_ind(id_pareto, "obj_space");
    cout << "............. " << dir << endl;
    // if (dir.size() == 0)
    // {
    //     // dirección del archivo
    //     dir = root + "/" + req.user_name + "/pareto_fronts/front" + id_pareto + "/optimization/obj_space.out";
    //     if (!is_file(dir))
    //     {
    //         cerr << "No se pudo localizar el espacio de los objetivos del individuo: " << id_pareto << endl;
    //         return false;
    //     }
    // }

    // abrimos el archivo
    string txt;
    fstream file_ind;
    file_ind.open(dir, std::ios_base::in);
    if (file_ind)
    {
        // leemos todo el contenido del archivo
        ostringstream os;
        os << file_ind.rdbuf();
        txt = os.str();
        // cout << txt << endl;

        // convertimos el texto a números
        vector<double> values = string_to_values(txt);
        cout << "size array: " << values.size() << endl;

        res.obj_space = values;
        // se sabe apriori el tamaño de objetivos
        res.num_obj = num_obj;

        if (values.size() % res.num_obj == 0)
        {
            res.pop_size = values.size() / res.num_obj;
        }
        else
        {
            cerr << "Error al cargar el espacio de los objetivos" << endl;
        }

        // res.obj_space.insert(res.obj_space.begin(), obj_space.begin()+2, obj_space.end());
        cout << "---------- Espacio de los objetivos cargado con éxito ---------------\n";
        return true;
    }
}

bool load_individual_var_space(neurocontroller_database::load_ind::Request &req,
                               neurocontroller_database::load_ind::Response &res)
{

    cout << "-----------  Cargando individuo " << req.num_ind << " -----------\n";
    // num neuronas de la red
    int n_in = 50;
    int n_out = 2;

    string dir_file = get_path_optimized_ind(req.id_pareto_front, "var_space");
    if (dir_file.size() < 0)
    {
        cout << "Solo se ha implementado la carga de individuos para los pregenerados con el optimizador\n";
        return false;
    }

    // se verifica que exista el archivo
    if (is_file(dir_file))
    {
        // abrimos el archivo
        fstream file_ind;
        file_ind.open(dir_file, std::ios_base::in);
        if (file_ind)
        {
            // nos posicionamos en la linea n
            int num_line = req.num_ind;
            file_ind.seekg(std::ios::beg);

            if (num_line > 0)
            {
                for (int i = 0; i < num_line; ++i)
                {
                    file_ind.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                }
            }

            // se extrae la línea del individuo seleccionado
            std::string txt;
            std::getline(file_ind, txt);
            file_ind.close();

            // stringstream data;
            // data << txt;

            vector<double> num = string_to_values(txt);
            // double n;
            // for(int i = 0; i < n_in*n_out; i++){
            //     data >> n;
            //     num.push_back(n);
            //     // cout << std::setprecision(12) << n << endl;
            // }
            res.var_space = num;
            return true;
        }
        else
        {
            cout << "No se pudo abrir el archivo: " + dir_file << endl;
            return false;
        }
    }
    else
    {
        cout << "No se encuentra el individuo: " + dir_file << endl;
        return false;
    }
}

bool do_id(neurocontroller_database::get_id::Request &req,
           neurocontroller_database::get_id::Response &res)
{
    // se buscan los archivos de señales  EEG y se extrae el número mayor
    int max = 0;
    for (boost::filesystem::directory_entry &entry : boost::filesystem::directory_iterator(req.directory))
    {
        string directory = entry.path().string();

        std::size_t found = directory.find(req.key);
        if (found != std::string::npos)
        {
            string str_num = directory.substr(found + req.key.size());

            // se extrae el número
            stringstream ss;
            ss << str_num;
            int num;
            ss >> num;

            if (max < num)
                max = num;
        }
    }

    // se crea el archivo
    //  fstream file_out;
    string id_signal = req.key + to_string(max + 1);

    res.id = id_signal;
    res.num = max + 1;

    return true;
}

bool type_id(neurocontroller_database::get_type_id::Request &req,
             neurocontroller_database::get_type_id::Response &res)
{

    std::size_t found = req.id_pareto_front.find("optimized");
    std::size_t found2 = req.id_pareto_front.find("optimized_basal");
    if (found != std::string::npos || found2 != std::string::npos)
    {
        res.type = "data_optimized_individuals";
    }
    else
    {
        found = req.id_pareto_front.find("root-");
        if (found != std::string::npos)
        {
            res.type = "pareto_fronts";
        }
        else
        {
            found = req.id_pareto_front.find("protocol");
            if (found != std::string::npos)
            {
                res.type = "protocol";
            }
            else
            {
                res.type = "unknown";
            }
        }
    }

    return true;
}

int main(int argc, char **argv)
{
    create_dir(root);

    ros::init(argc, argv, "data_server");
    ros::NodeHandle n;
    ros::ServiceServer srv_ind = n.advertiseService<neurocontroller_database::load_ind::Request, neurocontroller_database::load_ind::Response>("load_ind_var_space", load_individual_var_space);
    ros::ServiceServer srv_phylo = n.advertiseService<neurocontroller_database::load_phylogenetic::Request, neurocontroller_database::load_phylogenetic::Response>("load_phylogenetic", load_phylogenetic);
    ros::ServiceServer srv_save_phylo = n.advertiseService<neurocontroller_database::save_phylogenetic::Request, neurocontroller_database::save_phylogenetic::Response>("save_phylogenetic", save_phylogenetic);
    ros::ServiceServer srv_load_obj = n.advertiseService<neurocontroller_database::load_obj_space::Request, neurocontroller_database::load_obj_space::Response>("load_obj_space", load_obj_space_by_front);
    ros::ServiceServer srv_save_str = n.advertiseService<neurocontroller_database::save_data_string::Request, neurocontroller_database::save_data_string::Response>("save_data_string", save_str);
    ros::ServiceServer g_id = n.advertiseService<neurocontroller_database::get_id::Request, neurocontroller_database::get_id::Response>("get_id", do_id);
    ros::ServiceServer g_t_id = n.advertiseService<neurocontroller_database::get_type_id::Request, neurocontroller_database::get_type_id::Response>("get_type_id", type_id);

    // ros::ServiceServer srv_save = n.advertiseService<neurocontroller_database::save_pop::Request, neurocontroller_database::save_pop::Response>("save_population", save_population);
    // ros::ServiceServer srv_load = n.advertiseService<neurocontroller_database::load_pop::Request, neurocontroller_database::load_pop::Response>("load_population", load_population);

    cout << "Nodo server_db iniciado con éxito" << endl;
    ros::spin();
    return 1;
}

// bool load_population(neurocontroller_database::load_pop::Request &req, neurocontroller_database::load_pop::Response &res)
// {
//     cout << "\n\n----------------------- Cargando población -----------------------\n";
//     cout << "id pareto: " << req.id_pareto_front << endl;
//     cout << "\n\n";

//     // dir de los parámetros
//     string id_pareto = req.id_pareto_front;
//     string dir = root + "/" + req.user_name + "/pareto_fronts/front" + id_pareto + "/optimization";
//     // se verifica que exista el frente
//     if (!is_dir)
//     {
//         cout << "++++++++++++ frente de pareto: " << id_pareto << " no existe, se ha buscado en " << dir << " ++++++++++++\n";
//         return false;
//     }
//     // se leen los parámetros
//     ifstream file(dir + "/meta_data_optimizer.txt");
//     string txt;
//     while (getline(file, txt))
//     {
//         std::size_t pos = txt.find(":");
//         if (pos != std::string::npos)
//         {
//             string name = txt.substr(0, pos);
//             string data = txt.substr(pos + 1);
//             cout << name << ": " << data << endl;

//             if (name.find("pop_size") != std::string::npos)
//                 res.pop_size = std::stof(data);
//             if (name.find("n_gen") != std::string::npos)
//                 res.n_gen = std::stof(data);
//             if (name.find("num_obj") != std::string::npos)
//                 res.num_obj = std::stof(data);
//             if (name.find("id_pareto") != std::string::npos)
//                 res.id_pareto_front = data;
//             if (name.find("shape_ann") != std::string::npos)
//             {
//                 vector<double> shape = get_numbers(data);
//                 vector<int> shape_ann(shape.begin(), shape.end());
//                 res.shape_ann = shape_ann;
//             }
//             // if(name.find("num_var") != std::string::npos) res.num_var = std::stof(data);
//             if (name.find("num_constr") != std::string::npos)
//                 res.num_constr = std::stof(data);
//         }
//     }

//     // espacio de las variables y objetivos
//     vector<double> var_space = read_file(dir + "/var_space.csv");
//     vector<double> obj_space = read_file(dir + "/obj_space.csv");
//     vector<double> constraints = read_file(dir + "/constraints.csv");
//     cout << "Tamaños -- obj_space:" << var_space.size() << "  var_space:" << obj_space.size() << "  constr:" << constraints.size() << endl;
//     // el mas 2 es para evitar la shape del dataset
//     res.var_space.insert(res.var_space.end(), var_space.begin() + 2, var_space.end());
//     res.obj_space.insert(res.obj_space.end(), obj_space.begin() + 2, obj_space.end());
//     res.constraints.insert(res.constraints.end(), constraints.begin() + 2, constraints.end());

//     cout << "-------------------- población cargada con éxito -----------------------\n";
//     return true;
// }

// bool save_population(neurocontroller_database::save_pop::Request &req, neurocontroller_database::save_pop::Response &res)
// {
//     cout << "\n\n----------------------- Guardando población -----------------------\n";
//     cout << "Usuario :" << req.user_name << endl;
//     cout << "pop size: " << req.pop_size << endl;
//     cout << "var space: " << req.var_space.size() << endl;
//     cout << "obj space: " << req.obj_space.size() << endl;
//     cout << "n obj: " << req.num_obj << endl;
//     cout << "n gen: " << req.n_gen << endl;
//     cout << "n pareto: " << req.id_pareto_front << endl;

//     // tamaño del individuo
//     int n_neurons;
//     ros::NodeHandle nh;
//     ros::ServiceClient srv_neuro = nh.serviceClient<my_node::get_neurocontroller_data>("get_neurocontroller_data");

//     my_node::get_neurocontroller_data param;
//     if (srv_neuro.call(param))
//     {
//         n_neurons = param.response.num_obj_space;
//     }
//     else
//     {
//         cout << "Error al obtener el tamaño de población\n";
//         return false;
//     }
//     // int n_neurons = get_num_weights(req.shape_ann);
//     // int n_neurons = 90;
//     ofstream file;
//     ofstream file2;
//     ofstream file3;
//     // se crea la direccion a guardar la poblacion
//     string dir_save = root + "/" + req.user_name + "/pareto_fronts/front" + req.id_pareto_front + "/optimization";
//     if (!create_dir(dir_save))
//     {
//         ROS_ERROR("--- Error al crear el directorio de la población ----\n");
//         return false;
//     }

//     // direccion de los archivos de la poblacion
//     std::string path = dir_save + "/var_space.csv";
//     std::string path2 = dir_save + "/obj_space.csv";
//     std::string path3 = dir_save + "/constraints.csv";
//     file.open(path);
//     file2.open(path2);
//     file3.open(path3);

//     // shape de los obj
//     file2 << "# " << req.num_obj << " " << req.pop_size << "\n";
//     // shape constr
//     file3 << "# " << req.num_constr << " " << req.pop_size << "\n";

//     // 1d to 2d
//     int pop_size = req.pop_size;
//     vector<double> aux = req.var_space;
//     vector<double> aux2 = req.obj_space;
//     vector<double> aux3 = req.constraints;
//     for (int n_ind = 0; n_ind < pop_size; n_ind++)
//     {
//         // se extrae el individuo
//         auto p1 = aux.begin() + n_ind * n_neurons;
//         auto p2 = aux.begin() + (n_ind + 1) * n_neurons;
//         std::vector<double> indi(p1, p2);

//         // se extraen los objetivos
//         p1 = aux2.begin() + n_ind * req.num_obj;
//         p2 = aux2.begin() + (n_ind + 1) * req.num_obj;
//         std::vector<double> indi2(p1, p2);

//         // se extraen los constraints
//         p1 = aux3.begin() + n_ind * req.num_constr;
//         p2 = aux3.begin() + (n_ind + 1) * req.num_constr;
//         std::vector<double> indi3(p1, p2);

//         for (int i = 0; i < indi.size(); i++)
//             file << indi[i] << " ";
//         file << "\n";
//         for (int i = 0; i < indi2.size(); i++)
//             file2 << indi2[i] << " ";
//         file2 << "\n";
//         for (int i = 0; i < indi3.size(); i++)
//             file3 << indi3[i] << " ";
//         file3 << "\n";
//     }
//     file.close();
//     file2.close();
//     file3.close();

//     // se guardan los valores de los parámetros
//     string path_param = dir_save + "/meta_data_optimizer.txt";
//     ofstream f_param;
//     f_param.open(path_param);
//     f_param << "pop_size:" + to_string(req.pop_size) + "\n";
//     f_param << "n_gen:" + to_string(req.n_gen) + "\n";
//     f_param << "num_obj:" + to_string(req.num_obj) + "\n";
//     f_param << "id_pareto_front:" + req.id_pareto_front + "\n";
//     f_param << "num_constr:" << req.num_constr << "\n";
//     f_param.close();

//     cout << "------------ Población guardada con éxito ------------\n";
//     return true;
// }