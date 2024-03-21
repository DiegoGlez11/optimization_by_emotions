
module.exports = {


    get_obj_space: function (id_pareto, non_dominated, normalization, id_pareto_inds, num_indiviuals) {
        return new Promise((resolve, reject) => {
            let srv_obj = rosnodejs.nh.serviceClient("/load_object_space", "neurocontroller_database/load_obj_space");
            let param = new neurocontroller_database.load_obj_space.Request();
            param.id_pareto_front = id_pareto;
            param.normalize = normalization;
            param.non_dominated = non_dominated;

            srv_obj.call(param).then((res) => {
                let obj_space = res.obj_space;
                let num_obj = res.num_obj;

                let relative_num_ind = res.relative_num_ind;
                let relative_id_pareto = res.relative_id_pareto;

                let data = {}, str_ind;
                data["pop_size"] = res.pop_size;
                data["num_obj"] = num_obj;
                data["relative_num_ind"] = [];
                data["relative_id_pareto"] = [];
                data["obj_space"] = [];
                data["pos_id"] = {};

                // save information of individual
                let save_data = function (id, num, val) {
                    // num ind to string
                    str_ind = `ind_${num}`;

                    // pos of individual
                    if (data["pos_id"][id] == undefined) data["pos_id"][id] = {};
                    if (data["pos_id"][id][str_ind] == undefined) data["pos_id"][id][str_ind] = {};
                    data["pos_id"][id][str_ind] = data["relative_num_ind"].length;

                    // relative values
                    data["relative_num_ind"].push(num);
                    data["relative_id_pareto"].push(id);

                    // obj space with form num_obj*pop_size
                    for (let i = 0; i < val.length; i++) {
                        if (!Array.isArray(data["obj_space"][i]))
                            data["obj_space"][i] = [];

                        data["obj_space"][i].push(val[i]);
                    }
                };

                for (let n = 0; n < res.pop_size; n++) {
                    // extract ind
                    let ind = obj_space.splice(0, num_obj);
                    let id_front = relative_id_pareto.splice(0, 1)[0];
                    let num_ind = relative_num_ind.splice(0, 1)[0]

                    // if is necessary to get specific individuals
                    if (num_indiviuals != undefined && id_pareto_inds != undefined) {
                        // verify into list if is ind desired
                        for (let i = 0; i < num_indiviuals.length; i++) {
                            if (num_indiviuals[i] == num_ind && id_pareto_inds[i] == id_front) {
                                // erase ind
                                num_indiviuals.splice(i, 1);
                                id_pareto_inds.splice(i, 1);
                                // save data
                                save_data(id_front, num_ind, ind);
                            }
                        }
                    } else {
                        save_data(id_front, num_ind, ind);
                    }
                }
                // update pop size
                data["pop_size"] = data["relative_num_ind"].length;

                resolve(data);
            }).catch((e) => {
                console.error("Error al obtener el espacio de los objetivos\n\n", e);
                reject(e);
            })
        });
    },

    save_object: function (obj, name_file, dir_) {
        return new Promise((res, rej) => {
            save_data_string(JSON.stringify(obj), name_file, dir_).then((e) => {
                res(e);
            }).catch((g) => {
                console.error("Error al guardar el archivo:", dir_ + "/" + name_file, "\n\n", g);
                rej(g);
            });
        });
    },

    get_type_id: function (id_pareto) {
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
    },

    run_command: function (command) {
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
    },

    simulate_individual: function (id_pareto, n_child) {
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
                        // CONTROL_DATA.select_ind = false;
                        resolve();
                        // return;
                    });
                });
            }).catch((e) => {
                console.error(e);
            });
        });
    },
};


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

// ------------------------------------------------------------
// ------------------------------------------------------------
// ------------------------------------------------------------
// ------------------------------------------------------------
// ------------------------------------------------------------
// ------------------------------------------------------------
// ------------------------------------------------------------
// ------------------------------------------------------------



// function simulate_individual(id_pareto, n_child) {
//     return new Promise((resolve, reject) => {

//         // se cargan los valores de la simulación
//         load_object(`${root_dir}/control_params.txt`).then((neuro_param) => {

//             console.log(neuro_param, neuro_param.contact_thresold);
//             let err = `Error al cargar los parámetros de la simulacion ${neuro_param}`;

//             if (neuro_param["contact_thresold"] == undefined && neuro_param["simulation_time"] == undefined) {
//                 console.error(err);
//                 reject();
//                 return;
//             }
//             if (neuro_param["contact_thresold"] <= 0 && neuro_param["simulation_time"] <= 0) {
//                 console.error(err);
//                 reject();
//                 return;
//             }

//             let client_srv_evaluateDriver = rosnodejs.nh.serviceClient("/evaluate_driver", "arlo_controller_dmaking/EvaluateDriver");
//             let req_eval = new arlo_controller_dmaking.EvaluateDriver.Request();
//             req_eval.num_ind = n_child;
//             req_eval.weightsfile = id_pareto;
//             req_eval.maxtime = neuro_param.simulation_time;
//             req_eval.touchthreshold = neuro_param.contact_thresold;
//             console.log(req_eval);

//             client_srv_evaluateDriver.call(req_eval).then((res) => {
//                 resolve(res)
//             }).catch((e) => {
//                 // console.error("Error al simular el individuo. ID:" + id_pareto + " Num_ind:" + n_child, e);
//                 // reject(e);

//                 console.log("No se pudo iniciar la simulación, se usa tiempo para simular que se inició el robot en gazebo");
//                 sleep(2000).then(() => {
//                     resolve();
//                     return;
//                 });
//             });
//         }).catch((e) => {
//             console.error(e);
//         });
//     });

// }


// function simulate_individual(id_pareto, n_child) {
//     return new Promise((resolve, reject) => {

//         // se cargan los valores de la simulación
//         load_object(`${root_dir}/control_params.txt`).then((neuro_param) => {

//             let err = `Error al cargar los parámetros de la simulacion ${neuro_param}`;

//             if (neuro_param["contact_thresold"] == undefined && neuro_param["simulation_time"] == undefined) {
//                 console.error(err);
//                 reject();
//                 return;
//             }
//             if (neuro_param["contact_thresold"] <= 0 && neuro_param["simulation_time"] <= 0) {
//                 console.error(err);
//                 reject();
//                 return;
//             }

//             let client_srv_evaluateDriver = rosnodejs.nh.serviceClient("/evaluate_driver", "arlo_controller_dmaking/EvaluateDriver");

//             let req_eval = new arlo_controller_dmaking.EvaluateDriver.Request();
//             req_eval.num_ind = n_child;
//             req_eval.weightsfile = id_pareto;
//             req_eval.maxtime = neuro_param.simulation_time;
//             req_eval.touchthreshold = neuro_param.contact_thresold;

//             client_srv_evaluateDriver.call(req_eval).then((res) => {
//                 resolve(res)
//             }).catch((e) => {

//                 console.log("No se pudo iniciar la simulación, se usa tiempo para simular que se inició el robot en gazebo");
//                 sleep(2000).then(() => {
//                     resolve();
//                     // return;
//                 });
//             });
//         }).catch((e) => {
//             console.error(e);
//         });
//     });
// }




// // ------------------------------------------------------------
// // ------------------------------------------------------------
// // ------------------------------------------------------------
// // ------------------------------------------------------------
// // ------------------------------------------------------------
// // ------------------------------------------------------------
// // ------------------------------------------------------------
// // ------------------------------------------------------------

