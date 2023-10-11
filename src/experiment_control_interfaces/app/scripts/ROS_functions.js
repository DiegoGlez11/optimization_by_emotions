/*
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
+++++++++ Sericios ros++++++++++
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*/

function simulate_individual(id_pareto, n_child) {
    return new Promise((resolve, reject) => {

        // se cargan los valores de la simulación
        load_object(`${root_dir}/control_params.txt`).then((neuro_param) => {

            console.log(neuro_param, neuro_param.contact_thresold);
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
            console.log(req_eval);

            client_srv_evaluateDriver.call(req_eval).then((res) => {
                resolve(res)
            }).catch((e) => {
                // console.error("Error al simular el individuo. ID:" + id_pareto + " Num_ind:" + n_child, e);
                // reject(e);

                console.log("No se pudo iniciar la simulación, se usa tiempo para simular que se inició el robot en gazebo");
                sleep(2000).then(() => {
                    resolve();
                    return;
                });
            });
        }).catch((e) => {
            console.error(e);
        });
    });
}




// function load_phylogenetic_data(id_pareto) {
//     return new Promise((resolve, reject) => {
//         let srv_phylo = rosnodejs.nh.serviceClient("/load_phylogenetic", "neurocontroller_database/load_phylogenetic");

//         mp = Object.assign({}, srv_load_phylogenetic)
//         mp["user_name"] = get_actual_user();
//         mp["id_pareto_front"] = id_pareto;

//         srv_phylo.call(mp).then((r) => {
//             if (r.phylogenetic == "") resolve("");

//             let obj = JSON.parse(r.phylogenetic);
//             obj.pos_database = -1;
//             resolve(obj);
//         }).catch((e) => {
//             console.error("Error al cargar la información filogenética. ID: " + id_pareto);
//             reject(e);
//         });
//     });
// }


// //-------------------------------
// function save_phylogenetic_data(id_pareto, data) {
//     return new Promise((resolve, reject) => {
//         let srv_save = rosnodejs.nh.serviceClient("/save_phylogenetic", "neurocontroller_database/save_phylogenetic");
//         let msg_c = Object.assign({}, srv_save_population_nsga2);
//         msg_c.phylogenetic = JSON.stringify(data);
//         msg_c.user_name = get_actual_user();
//         msg_c.id_pareto_front = id_pareto;
//         srv_save.call(msg_c).then((res) => {
//             resolve(res);
//         }).catch((e) => {
//             console.error("Error al guardar la información filogenética");
//             reject(e);
//         });
//     });
// }

function start_protocol(control) {
    return new Promise((resolve, reject) => {
        // start_protocol, resume_protocol
        let srv = rosnodejs.nh.serviceClient("/control_protocol", "emotion_evocation_protocol/control_protocol");

        let param = new emotion_evocation_protocol.control_protocol.Request();
        param.control = control;

        srv.call(param).then(() => {
            resolve();
        }).catch((e) => {
            console.error("Error con el protocolo");
            reject(e);
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




function control_emotion(param) {//(window_size, stride, model_name, k_rep, num_fold, num_epoch, batch_size, tolerance) {
    return new Promise(async (resolve, reject) => {
        let success = 0;

        // let param = new emotion_classification.control_emotions.Request();
        // param.window_size = window_size;
        // param.stride = stride;

        // control de clasificador
        let srv1 = rosnodejs.nh.serviceClient("/control_emotions", "emotion_classification/control_emotions");
        await srv1.call(param).then(() => {
            success++;
        }).catch((e) => {
            console.error("Error al controlar el nodo de clasificación\n");
        });

        // control del entrenamiento
        // param.model_name = model_name;
        // param.k_repetitions = k_rep;
        // param.size_fold = num_fold;
        // param.num_epoch = num_epoch;
        // param.batch_size = batch_size;
        // param.tolerance = tolerance;
        let srv2 = rosnodejs.nh.serviceClient("/control_emotions_train", "emotion_classification/control_emotions");
        await srv2.call(param).then(() => {
            success++;
        }).catch((e) => {
            console.error("Error al controlar el nodo de entrenamiento\n");
        });

        if (success > 0) resolve();
        else reject();
    });
}

function control_acquisitionEEG(param) {
    return new Promise((resolve, reject) => {
        let srv_ctl = rosnodejs.nh.serviceClient("/control_acquisition", "eeg_signal_acquisition/control_acquisition");

        srv_ctl.call(param).then((res) => {
            resolve(res);
        }).catch((e) => {
            console.error("Error al conectar el control de la adquisicion");
            reject(e);
        });
    });
}


function get_state_acquisition() {
    return new Promise((resolve, reject) => {
        // si se usa el modo SAM
        if (document.getElementById("rad_sam").checked) {
            resolve({ connected: false });
            return;
        }

        // Modo BCI
        if (document.getElementById("rad_bci").checked) {

            let srv_conn = rosnodejs.nh.serviceClient("/get_state_acquisition", "eeg_signal_acquisition/get_state_acquisition");
            let srv_msg = new eeg_signal_acquisition.get_state_acquisition.Request();
            srv_conn.call(srv_msg).then((res) => {
                resolve(res);
            }).catch((e) => {
                console.error("Error al obtener el estado del nodo de adquisción");
                reject(e);
            });
        }
    });
}


function get_emotion_data() {
    return new Promise((resolve, reject) => {
        let srv = rosnodejs.nh.serviceClient("/get_emotion_data", "emotion_classification/get_emotion_data");
        let param = new emotion_classification.get_emotion_data.Request();;
        srv.call(param).then((res) => {
            resolve(res);
        }).catch((e) => {
            console.error("Error al obtener los datos metadatos del modelo emocional", e);
            reject();
        });
    });
}


function get_model_instancies(model_name) {
    return new Promise((resolve, reject) => {
        let srv = rosnodejs.nh.serviceClient("/get_instancies_model_name", "emotion_classification/get_instancies_model_name")
        let param = new emotion_classification.get_instancies_model_name.Request();
        param.user_name = "diego"
        param.model_name = model_name;

        srv.call(param).then((res) => {
            resolve(res);
        }).catch((e) => {
            console.error("Error al obtener las instancia de " + model_name);
            reject(e);
        });
    });
}


function get_preprocessing_val() {
    return new Promise((resolve, reject) => {
        // si se usa el modo SAM
        if (document.getElementById("rad_bci").checked) {


            let srv = rosnodejs.nh.serviceClient("/get_preprocessing_values", "eeg_signal_acquisition/get_preprocessing_values");

            srv.call({}).then((res) => {
                resolve(res);
            }).catch((e) => {
                console.error("Error al obtener los datos del preprocesamiento");
                reject(e);
            });
        } else {
            resolve({});
        }
    });
}

function set_preprocessing_val(freq_ini, freq_end, filter_order, remove_env_noise, do_bandpass, do_detrend, do_save, do_reference_eeg, do_reference_eeg_common) {
    return new Promise((resolve, reject) => {
        let srv = rosnodejs.nh.serviceClient("/set_preprocessing_values", "eeg_signal_acquisition/set_preprocessing_values");
        let param = new eeg_signal_acquisition.set_preprocessing_values.Request();
        param.filter_order = filter_order;
        param.frequency_ini = freq_ini;
        param.frequency_end = freq_end;
        param.remove_environment_noise = remove_env_noise;
        param.perform_bandpass = do_bandpass;
        param.detrend = do_detrend;
        param.is_save = do_save;
        param.reference_eeg = do_reference_eeg;
        param.reference_eeg_common = do_reference_eeg_common;

        srv.call(param).then((res) => {
            resolve(res);
        }).catch((e) => {
            console.error("Error al obtener los datos del preprocesamiento");
            reject(e);
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
            console.error("Error al obtener el tipo de ID", e);
            reject(e);
        });
    });
}

function get_evaluations_srv(user_name, model_name, id_emotion_model, num_rep, num_fold) {
    return new Promise((resolve, reject) => {
        let srv = rosnodejs.nh.serviceClient("/get_emotion_evaluations", "neurocontroller_database/get_emotion_evaluations");
        let param = new neurocontroller_database.get_emotion_evaluations.Request();
        param.user_name = user_name;
        param.model_name = model_name;
        param.id_emotion_model = id_emotion_model;
        param.num_rep = num_rep;
        param.num_fold = num_fold;

        srv.call(param).then((eval_m) => {
            let str_eval = eval_m["evaluations"];
            if (str_eval == "") {
                reject();
                return;
            }
            // console.log(str_eval);
            let evals = JSON.parse(str_eval);
            resolve(evals);
        }).catch((e) => {
            console.error("Error al obtener los parámetros del preprocesamiento\n\n");
            reject(e);
        });
    });
}

function load_predictions(param) {
    return new Promise((resolve, reject) => {
        let srv = rosnodejs.nh.serviceClient("/get_emotion_predictions", "neurocontroller_database/get_emotion_evaluations");

        srv.call(param).then((eval_m) => {
            let str_eval = eval_m["evaluations"];
            if (str_eval == "") {
                reject();
                return;
            }
            // console.log(str_eval);
            let evals = JSON.parse(str_eval);
            resolve(evals);
        });
    });
}

// function get_emotion_dist(id_pareto, num_ind) {
//     return new Promise(() => {
//         let srv = rosnodejs.nh.serviceClient("/get_emotional_distributions", "neurocontroller_database/get_emotional_distributions");
//         let param = new neurocontroller_database.get_emotional_distributions.Request();
//         param.user_name = get_actual_user();
//         param.id_pareto_front = id_pareto;
//         param.num_ind = num_ind;
//         param.storage_pos = ;
//     });
// }


function get_obj_space(id_pareto) {
    return new Promise((resolve, reject) => {
        let srv_obj = rosnodejs.nh.serviceClient("/load_obj_space", "neurocontroller_database/load_obj_space");
        let param = new neurocontroller_database.load_obj_space.Request();
        param.id_pareto_front = id_pareto;

        srv_obj.call(param).then((res) => {
            resolve(res);
        }).catch((e) => {
            reject(e);
        })
    });
}

// plot de los frentes en la gráfica 3D
function plot_front(msg) { //(id_pareto, sel_inds = undefined, ref_point = undefined, num_ref = undefined, id_ref = undefined, close_point = undefined, num_close = undefined, id_close = undefined) {
    return new Promise((resolve, reject) => {
        let pub = rosnodejs.nh.serviceClient("/next_front_selected", "user_interaction/next_front_selected");
        // let msg = user_interaction_msg.next_front_ind();

        // msg.id_pareto_front = id_pareto;
        // if (sel_inds != undefined) msg.num_individuals = sel_inds;

        // if (ref_point != undefined) msg.reference_point = ref_point;
        // if (ref_point != undefined) msg.reference_point = ref_point;
        // if (ref_point != undefined) msg.reference_point = ref_point;

        // if (ref_point != undefined) msg.reference_point = ref_point;
        // if (ref_point != undefined) msg.reference_point = ref_point;
        // if (ref_point != undefined) msg.reference_point = ref_point;
        pub.call(msg).then(() => {
            resolve();
        }).catch((e) => {
            reject(e);
        });
        resolve();
    });
}


function experiment_control(param) {
    return new Promise((resolve, reject) => {
        let srv = rosnodejs.nh.serviceClient("experiment_control", "user_interaction/experiment_control");
        // let param = new user_interaction.experiment_control.Request();
        // param.start_effects_window = true;

        srv.call(param).then(() => {
            resolve();
        }).catch((e) => {
            console.error("ERROR AL ABRIR LA VENTANA DE EFECTOS\N");
            reject();
        });
    })
}


function ros_sub_change_transmit_state() {
    const sub = rosnodejs.nh.subscribe('/change_state_eeg_msg', 'experiment_control_interfaces/change_state_eeg_msg', (req) => {
        // console.log('Got msg on chatter: %j', req);

        if (req.is_capturing) {
            document.getElementById("transmit").className = "init";
        } else {
            document.getElementById("transmit").className = "stop";
        }

        if (req.is_connected) {
            document.getElementById("connected").className = "init";
        } else {
            document.getElementById("connected").className = "stop";
        }

        // console.log("-----------", req);
        return true;
    });
}


//++++++++++++++++++++++++++++++++++++++++++++++
ros_sub_change_transmit_state()
