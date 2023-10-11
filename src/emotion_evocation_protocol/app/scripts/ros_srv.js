


function play_eeg(id_pareto_front = "", num_ind = -1, storage_pos, is_save_file = true) {
    return new Promise((resolve, reject) => {
        //preguntamos por el estado del casco
        let srv_state = rosnodejs.nh.serviceClient("/get_state_acquisition", "eeg_signal_acquisition/get_state_acquisition");
        srv_state.call({}).then((res) => {
            if (!res.connected) {
                // if (show_msg)
                alert("No hay tarjeta conectada");
                reject();
                return;
            }

            //se inician las señales si esta detenido el flujo
            if (res.state == "stop") {
                //se inician las señales
                let srv = rosnodejs.nh.serviceClient("/start_acquisition_eeg", "eeg_signal_acquisition/start_acquisition_eeg");
                //mensaje para iniciar el flujo de señales eeg
                let msg_flow = new eeg_signal_acquisition.start_acquisition_eeg.Request();

                msg_flow.user_name = get_user();
                msg_flow.id_pareto_front = id_pareto_front;
                msg_flow.num_ind = num_ind;
                msg_flow.is_publish = false; // SOLO PARA EL PROTOCOLO
                msg_flow.is_save_file = is_save_file;
                msg_flow.storage_pos = storage_pos;

                srv.call(msg_flow).then((res2) => {
                    // let btn_aux = document.getElementById("btn_continuar");
                    // btn_aux.setAttribute("actual")
                    // document.getElementById("transmit").className = "init";
                    // update_btn("play");
                    resolve(res2.id_eeg_signal);
                }).catch((e) => {
                    console.error("Error al iniciar el fujo de señales EEG");
                    reject();
                });
            } else {
                // document.getElementById("transmit").className = "init";
                resolve();
            }
        }).catch((e) => {
            console.error("Error al obtener el estado del casco EEG");
            reject();
        });
    }).catch((e) => {
        reject(e);
    });
}



function stop_eeg() {
    return new Promise((resolve, reject) => {
        let srv = rosnodejs.nh.serviceClient("/stop_acquisition_eeg", "eeg_signal_acquisition/stop_acquisition_eeg");
        //se detiene flujo de señales eeg
        srv.call({}).then((res2) => {
            resolve(res2);
        }).catch((e) => {
            console.log(e);
            alert("Erro al iniciar el fujo de señales EEG: ");
            reject(e);
        });

    });
}




function create_id(key, dir) {
    return new Promise((resolve, reject) => {
        let srv = rosnodejs.nh.serviceClient("/get_id", "neurocontroller_database/get_id");
        let param = new neurocontroller_database.get_id.Request();
        param.key = key;
        param.directory = dir;

        srv.call(param).then((res) => {
            resolve(res);
        }).catch((e) => {
            console.error("Erro al obtener el ID", e);
            reject();
        });
    });
}


function save_data_string(str, name_file, dir_) {
    return new Promise((resolve, reject) => {
        let srv_str = rosnodejs.nh.serviceClient("/save_data_string", "neurocontroller_database/save_data_string");
        let srv_param = new neurocontroller_database.save_data_string.Request();
        srv_param.data_string = str;
        srv_param.directory = dir_;
        srv_param.name_file = name_file;

        srv_str.call(srv_param).then((res) => {
            resolve(res);
        }).catch((e) => {
            console.error("Error al guardar la cadena de texto");
            reject(e);
        });
    });
}


function save_object(obj, name_file, dir_) {
    return new Promise((res, rej) => {
        save_data_string(JSON.stringify(obj), name_file, dir_).then((e) => {
            res(e);
        }).catch((g) => {
            console.error("Error al guardar el archivo:", dir_ + "/" + name_file, "\n\n", g);
            rej(g);
        });
    });
}




function load_object(path_obj) {
    return new Promise((resolve, reject) => {
        fs.readFile(path_obj, 'utf8', (err, data) => {
            if (err) {

                resolve({});
            } else {
                let obj = {};
                // console.log(data);
                try {
                    if (data.length > 0) obj = JSON.parse(data);
                    resolve(obj);
                } catch (e) {
                    console.error("Error al cargar el archivo: ", path_obj, "\n\n", e);
                    resolve();
                }

            }
        });
    });
}



// Register node with ROS master
function init_ROS() {
    rosnodejs.initNode("gui_control_experiment").then((rosNode) => {
        rosnodejs.log.info("Nodo GUI creado con exito");
    }).catch((e) => {
        console.error("No se pudo crear el nodo ROS: " + e);
        alert("Sin conexion con la red ROS: " + e);
    });
}

init_ROS();