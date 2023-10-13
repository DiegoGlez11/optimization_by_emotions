/*
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
+++++++++ Servicios ros++++++++++
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*/
// function simulate_individual(id_pareto, n_child) {
//     return new Promise((resolve, reject) => {
//         // sleep(1000).then(() => {
//         //     resolve();
//         //     return;
//         // });

//         let client_srv_evaluateDriver = rosnodejs.nh.serviceClient("/evaluate_driver", "arlo_controller_dmaking/EvaluateDriver");
//         //input del servicio

//         let req_eval = new arlo_controller_dmaking.EvaluateDriver.Request();
//         req_eval.num_ind = n_child;
//         req_eval.weightsfile = id_pareto;
//         req_eval.maxtime = 20;
//         req_eval.touchthreshold = 0.95;

//         client_srv_evaluateDriver.call(req_eval).then((res) => {
//             resolve(res)
//         }).catch((e) => {
//             // console.error("Error al simular el individuo. ID:" + id_pareto + " Num_ind:" + n_child, e);
//             // reject(e);

//             console.log("No se pudo iniciar la simulación, se usa tiempo para simular que se inició el robot en gazebo");
//             sleep(2000).then(() => {
//                 resolve();
//                 return;
//             });
//         });
//     });
// }



function simulate_individual(id_pareto, n_child) {
    return new Promise((resolve, reject) => {

        // se cargan los valores de la simulación
        load_object(`${root_dir}/control_params.txt`).then((neuro_param) => {

            let err = `Error al cargar los parámetros de la simulacion ${neuro_param}`;

            if (neuro_param["contact_thresold"] == undefined && neuro_param["simulation_time"] == undefined) {
                console.error(err);
                reject();
                return;
            }
            if (neuro_param["contact_thresold"] <= 0 && neuro_param["simulation_time"] <= 0) {
                console.error(err);
                reject();
                return;
            }

            let client_srv_evaluateDriver = rosnodejs.nh.serviceClient("/evaluate_driver", "arlo_controller_dmaking/EvaluateDriver");

            let req_eval = new arlo_controller_dmaking.EvaluateDriver.Request();
            req_eval.num_ind = n_child;
            req_eval.weightsfile = id_pareto;
            req_eval.maxtime = neuro_param.simulation_time;
            req_eval.touchthreshold = neuro_param.contact_thresold;

            client_srv_evaluateDriver.call(req_eval).then((res) => {
                resolve(res)
            }).catch((e) => {

                console.log("No se pudo iniciar la simulación, se usa tiempo para simular que se inició el robot en gazebo");
                sleep(2000).then(() => {
                    resolve();
                    // return;
                });
            });
        }).catch((e) => {
            console.error(e);
        });
    });
}


function control_robot(control = "") {
    return new Promise((resolve, reject) => {
        let srv = rosnodejs.nh.serviceClient("/control_robot", "arlo_controller_dmaking/control_robot");
        let param = new arlo_controller_dmaking.control_robot.Request();
        param.control = control;
        srv.call(param).then((res) => {
            resolve(res);
        }).catch((e) => {
            // console.error("Error al controlar el robot");
            reject("Error al controlar el robot")
        });
    });

}



function sound_control(name) {
    return new Promise((resolve, reject) => {
        let srv = rosnodejs.nh.serviceClient("/play_song", "arlo_controller_dmaking/play_song");
        let param = new arlo_controller_dmaking.play_song.Request();
        param.name = name;

        if (name == "") {
            console.error("Error de control de sonido");
            reject();
        }

        srv.call(param).then(() => {
            resolve();
        }).catch((e) => {
            reject("Error con el sonido");
        })
    });
}



function pause_gazebo() {
    return new Promise((resolve, reject) => {
        let srv = rosnodejs.nh.serviceClient("/gazebo/pause_physics", "std_srvs/Empty");

        srv.call({}).then(() => {
            resolve();
        }).catch((e) => {
            console.error("Error al pausar el simulador");
            reject();
        })
    });
}

function play_gazebo() {
    return new Promise((resolve, reject) => {
        let srv = rosnodejs.nh.serviceClient("/gazebo/unpause_physics", "std_srvs/Empty");

        srv.call({}).then(() => {
            resolve();
        }).catch((e) => {
            // console.error("Error al iniciar la simulación");
            reject("Error al iniciar la simulación");
        })
    });
}



function save_data_string(str, name_file, dir_) {
    return new Promise((resolve, reject) => {
        // console.log(str, name_file, dir_);
        let srv_str = rosnodejs.nh.serviceClient("/save_data_string", "neurocontroller_database/save_data_string");
        let srv_param = new neurocontroller_database.save_data_string.Request();
        srv_param.data_string = str;
        srv_param.directory = dir_;
        srv_param.name_file = name_file;
        // console.log(srv_param);
        srv_str.call(srv_param).then((res) => {
            resolve(res);
        }).catch((e) => {
            if (is_conn_refused(e)) e = "";
            reject(`Error al guardar la cadena de texto ${name_file}\n\n${e}`);
        });
    });
}


// function get_window_size() {
//     return new Promise((resolve, reject) => {
//         let srv_win = rosnodejs.nh.serviceClient("/control_emotions", "emotion_classification/control_emotions");
//         let param = new emotion_classification.control_emotions.Request();
//         param.action = "get_window_size";

//         srv_win.call(param).then((res) => {
//             resolve(res.window_size);
//         }).catch((e) => {
//             console.error("Error al obtener el tamaño de ventana");
//             reject(e);
//         });
//     });
// }

// function get_stride_window() {
//     return new Promise((resolve, reject) => {
//         let srv_win = rosnodejs.nh.serviceClient("/control_emotions", "emotion_classification/control_emotions");
//         let param = new emotion_classification.control_emotions.Request();
//         param.action = "get_stride";

//         srv_win.call(param).then((res) => {
//             resolve(res.stride);
//         }).catch((e) => {
//             console.error("Error al obtener el tamaño de ventana");
//             reject(e);
//         });
//     });
// }


// function control_acquisitionEEG(param) {
//     return new Promise((resolve, reject) => {
//         let srv_ctl = rosnodejs.nh.serviceClient("/control_acquisition", "eeg_signal_acquisition/control_acquisition");

//         srv_ctl.call(param).then((res) => {
//             resolve(res);
//         }).catch((e) => {
//             console.error("Error al conectar el control de la adquisicion");
//             reject(e);
//         });
//     });
// }

// function get_sample_rate() {
//     return new Promise((resolve, reject) => {
//         let srv_param = new eeg_signal_acquisition.control_acquisition.Request();
//         srv_param.sampling_rate = true;

//         control_acquisitionEEG(srv_param).then((res) => {
//             resolve(res.sampling_rate);
//         }).catch((e) => {
//             console.error("Error al obtener la frecuencia de muestreo");
//             reject(e);
//         });
//     });
// }

function run_command(command) {
    return new Promise((resolve, reject) => {
        exec(command, function (error, stdout, stderr) {
            console.log('stdout: ' + stdout);
            console.log('stderr: ' + stderr);
            if (error !== null) {
                console.log('exec error: ' + error);
                reject(error)
            }
            resolve();
        });
    });
}


function get_obj_space(id_pareto, is_norm, non_dominated) {
    return new Promise((resolve, reject) => {
        let srv_obj = rosnodejs.nh.serviceClient("/load_object_space", "neurocontroller_database/load_obj_space");
        let param = new neurocontroller_database.load_obj_space.Request();
        param.id_pareto_front = id_pareto;
        param.normalize = is_norm;
        param.non_dominated = non_dominated;
        param.load_test = INTERFACE_TEST;

        srv_obj.call(param).then((res) => {
            resolve(res);
        }).catch((e) => {
            console.error("Error al obtener el espacio de los objetivos\n\n", e);
            reject(e);
        })
    });
}

// function get_num_segments(size) {
//     return new Promise((resolve, reject) => {
//         load_object(`${root_dir}/train_parameters.txt`).then((params) => {
//             console.log("param to calculate get_segs", params, "size", size);
//             let window_size = params["window_size"];
//             let stride = params["stride"];

//             let rest = size - window_size;
//             // no se genera un segmento
//             if (rest < 0) {
//                 resolve(0);
//             }

//             let segments = parseInt(rest / stride) + 1;
//             resolve(segments);

//         }).catch((e) => {
//             console.error(e);
//             reject();
//         });
//     });
// }

function play_eeg(id_pareto_front = "", num_ind = -1, storage_pos, is_save_file = true) {
    return new Promise((resolve, reject) => {
        // if (EMOTIONS_CAPTURE_MODE == "sam") {
        //     resolve();
        //     return;
        // }

        get_actual_user().then((user_name) => {

            //preguntamos por el estado del casco
            get_state_acquisition().then(() => {
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

                    msg_flow.user_name = user_name;
                    msg_flow.id_pareto_front = id_pareto_front;
                    msg_flow.num_ind = num_ind;
                    msg_flow.is_publish = true;
                    msg_flow.is_save_file = is_save_file;
                    msg_flow.storage_pos = storage_pos;

                    srv.call(msg_flow).then((res2) => {
                        resolve(res2.id_eeg_signal);
                    }).catch((e) => {
                        if (is_conn_refused(e)) e = "";
                        reject(`Error al iniciar el flujo de señales EEG\n\n${e}`);
                    });
                } else {
                    resolve();
                }
            }).catch((e) => {
                reject(e);
            });
        }).catch((e) => {
            reject(e);
        });
    });
}



function stop_eeg() {
    return new Promise((resolve, reject) => {
        // if (EMOTIONS_CAPTURE_MODE == "sam") {
        //     resolve();
        //     return;
        // }

        let srv = rosnodejs.nh.serviceClient("/stop_acquisition_eeg", "eeg_signal_acquisition/stop_acquisition_eeg");
        //se detiene flujo de señales eeg
        srv.call({}).then((res2) => {
            resolve(res2);
        }).catch((e) => {
            if (is_conn_refused(e)) e = "";
            reject(`Error al iniciar el flujo de señales EEG\n\n${e}`);
        });
    });
}



function get_type_id(id_pareto) {
    return new Promise((resolve, reject) => {
        let srv = rosnodejs.nh.serviceClient("/get_type_id", "neurocontroller_database/get_type_id");
        let param = new neurocontroller_database.get_type_id.Request();
        param.id_pareto_front = id_pareto;

        srv.call(param).then((res) => {
            resolve(res.type);
        }).catch((e) => {
            if (is_conn_refused(e)) e = "";
            reject(`Error al obtener el tipo de ID: ${id_pareto}\n\n${e}`);
        });
    });
}


function get_state_acquisition() {
    return new Promise((resolve, reject) => {
        // if (EMOTIONS_CAPTURE_MODE == "sam") {
        //     resolve({ connected: true });
        //     return;
        // }

        let srv_conn = rosnodejs.nh.serviceClient("/get_state_acquisition", "eeg_signal_acquisition/get_state_acquisition");
        let srv_msg = new eeg_signal_acquisition.get_state_acquisition.Request();
        srv_conn.call(srv_msg).then((res) => {
            resolve(res);
        }).catch((e) => {
            if (is_conn_refused(e)) e = "";
            reject(`Error al obtener el estado del nodo de adquisción\n\n${e}`);
        });
    });
}


// parámetros del módulo de preferencias
function change_model_preference(num_ind_front = undefined, threshold_front = undefined, reset_table = undefined, id_pareto = undefined, extend_search = undefined, test_interface = undefined) {
    return new Promise((resolve, reject) => {
        let srv_c = rosnodejs.nh.serviceClient("/control_preference", "user_interaction/control_preference");
        // usuario actual
        get_actual_user().then((user_name) => {
            let param = new user_interaction.control_preference.Request();
            param.user_name = user_name;
            if (num_ind_front != undefined) param.num_individuals_by_front = num_ind_front;
            if (threshold_front != undefined) param.search_threshold_front = threshold_front;
            if (reset_table != undefined) param.reset_table = reset_table;
            if (id_pareto != undefined) param.id_pareto_front = id_pareto;
            if (extend_search != undefined) {
                if (extend_search == true) param.control = "extend_search";
                if (extend_search == false) param.control = "non_extend_search";
            }
            if (test_interface == true) param.control = "load_test";
            if (test_interface == false) param.control = "deactiv_load_test";

            srv_c.call(param).then(() => {
                resolve();
            }).catch((e) => {
                console.error("Error al cambiar los parámetros del módulo de preferencias\n\n", e);
                reject();
            });
        }).catch((e) => {
            reject(e);
        });
    });
}


// cambia los parámetros del entrenamiento
function change_parameters_train(in_params) {
    return new Promise((resolve, reject) => {
        let srv = rosnodejs.nh.serviceClient("/control_emotion_train", "emotion_classification/control_emotions");
        let params = new emotion_classification.control_emotions.Request();

        if (in_params["model_name"] != undefined) params["model_name"] = in_params["model_name"];
        if (in_params["window_size"] != undefined) params["window_size"] = in_params["window_size"];
        if (in_params["stride"] != undefined) params["stride"] = in_params["stride"];
        if (in_params["num_rep"] != undefined) params["k_repetitions"] = in_params["num_rep"];
        if (in_params["num_fold"] != undefined) params["size_fold"] = in_params["num_fold"];
        if (in_params["num_epoch"] != undefined) params["num_epoch"] = in_params["num_epoch"];
        if (in_params["tolerance"] != undefined) params["tolerance"] = in_params["tolerance"];
        if (in_params["batch_size"] != undefined) params["batch_size"] = in_params["batch_size"];

        srv.call(params).then(() => {
            // console.log("...........");
            resolve();
        }).catch(() => {
            console.error("Error al configurar los parámetros del entrenamiento\n\n");
            reject();
        });

    });
}



function search_new_front(user_name, prefered_ind, id_pareto, num_ind, storage_pos) {
    return new Promise((resolve, reject) => {
        let srv = rosnodejs.nh.serviceClient("/search_pareto_front", "user_interaction/search_pareto_front");
        let param = new user_interaction.search_pareto_front.Request();
        param.user_name = user_name;
        param.reference_point = prefered_ind;
        param.id_pareto_front = id_pareto;
        param.num_ind = num_ind;
        param.storage_position = storage_pos;

        srv.call(param).then(() => {
            resolve();
        }).catch((e) => {
            if (is_conn_refused(e)) e = "";
            reject(`Error al buscar un nuevo frente\n\n${e}`)
        });
    });
}