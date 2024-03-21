


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