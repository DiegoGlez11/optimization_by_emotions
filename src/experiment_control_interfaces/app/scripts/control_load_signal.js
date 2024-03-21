// SCRIPT DE LOS CONTROLES DE CARGA DE LA SEÑAL


// devuelve los subdirectorios de una ruta
function get_subdirs(dir_) {
    return new Promise((resolve, reject) => {
        fs.readdir(dir_, { withFileTypes: true }, (error, files) => {
            if (error) {
                // console.error(error);
                resolve([]);
                return;
            }

            const directoriesInDIrectory = files.filter((item) => {
                //se buscan todas las poblaciones root
                // if (item.name.search("root") > -1) {
                //     return item.isDirectory();
                // }
                return item.isDirectory();
            }).map((item) => item.name);

            resolve(directoriesInDIrectory);
        });
    });
};



function update_from_dir(dir, sel, p = false, call_back = undefined) {
    return new Promise((resolve, reject) => {
        if (!p) {
            resolve();
            return;
        }

        // se obtienen las subcarpetas
        get_subdirs(dir).then((sub_dir) => {
            sel.innerHTML = "";

            if (sub_dir.length == 0) return;

            for (let i = 0; i < sub_dir.length; i++) {
                let sub = sub_dir[i];
                // se carga el usuario
                let opt = document.createElement("option");
                opt.innerHTML = sub;
                opt.value = sub;

                if (call_back != undefined) {
                    // console.log(call_back);
                    let res_opt = call_back(opt);
                    if (res_opt != undefined) sel.append(opt);
                } else {
                    sel.append(opt);
                }
            }
            resolve();
        });
    });
}


function update_sel(name_sel) {
    return new Promise((resolve, reject) => {
        let pos = 0;
        let dir_obj = root_dir;

        if (name_sel == "user_name") pos = 1;
        if (name_sel == "type_id") pos = 2;
        if (name_sel == "id_experiment") pos = 3;
        if (name_sel == "num_ind") pos = 4;
        if (name_sel == "id_eeg_signal") pos = 5;

        let call_back = function (opt) {
            if (opt.value != "deap" && opt.value != "optimized_individuals") {
                // if (opt.value == "diego") opt.selected = "selected";
                return opt;
            } else {
                return undefined;
            }
        }

        let call_back2 = function (opt) {
            if (opt.value == "history") return undefined;
            else return opt;
        }

        let p1 = p2 = p3 = true;

        if (pos >= 1) p1 = false;
        if (pos >= 3) p2 = false;
        if (pos >= 4) p3 = false;
        // root
        update_from_dir(dir_obj, sel_user, p1, call_back).then(() => {
            // nombre de usuario seleccionado
            dir_obj += "/" + get_selected(sel_user);
            // tipo de id
            let t = get_selected(sel_type_exp);
            COMM_FUNCT.get_type_id(t).then((type) => {
                dir_obj += "/" + type;
                update_from_dir(dir_obj, sel_id_experiment, p2, call_back2).then(() => {
                    //ID experimento
                    dir_obj += "/" + get_selected(sel_id_experiment);
                    update_from_dir(dir_obj, sel_num_ind, p3).then(() => {
                        // num de individuo
                        dir_obj += "/" + get_selected(sel_num_ind);
                        // se obtienen los nombres de los archivos
                        fs.readdir(dir_obj, (err, files) => {
                            sel_id_signal.innerHTML = "";
                            files.forEach(file => {
                                //nombre de archivo válido
                                if (file.search("estimulation") >= 0 || file.search("simulation") >= 0) {
                                    let pos_ = file.search(".csv");
                                    if (pos_ >= 0) file = file.slice(0, pos_);
                                    let op = document.createElement("option");
                                    op.value = file;
                                    op.innerHTML = file;
                                    sel_id_signal.append(op);
                                }
                            });
                        });
                    });
                });
            });
        });
    });
}

let sel_user = document.getElementById("sel_user_name");
let sel_type_exp = document.getElementById("sel_type_exp");
let sel_id_experiment = document.getElementById("sel_id_experiment");
let sel_num_ind = document.getElementById("sel_num_ind");
let sel_id_signal = document.getElementById("sel_id_eeg_signal");

// select inicial
update_sel("");


// cambios en el sel del usuario
sel_user.onchange = () => {
    update_sel("user_name");
};

// select del id de experimento
sel_id_experiment.onchange = () => {
    update_sel("id_experiment");
};

sel_type_exp.onchange = () => {
    update_sel("type_id");
};

sel_num_ind.onchange = () => {
    update_sel("num_ind");
};



let btn_send = document.getElementById("btn_load_dataset");
btn_send.addEventListener("click", () => {
    // load_dataset(get_selected(sel_user), get_selected(sel_id_experiment)
    //     , get_selected(sel_num_ind), get_selected(sel_id_signal));

    preporcessing_signal(get_selected(sel_user), get_selected(sel_id_experiment)
        , get_selected(sel_num_ind), get_selected(sel_id_signal), 0);
});


function preporcessing_signal(user_name, id_pareto, num_ind, id_signal, storage_pos) {
    return new Promise((resolve, reject) => {
        let srv = rosnodejs.nh.serviceClient("/preprocessing_signal", "eeg_signal_acquisition/eeg_block_srv");
        let param = new eeg_signal_acquisition.eeg_block_srv.Request();

        param.user_name = user_name;
        param.id_pareto_front = id_pareto;
        param.num_ind = num_ind;
        param.id_eeg_signal = id_signal;
        param.storage_pos = storage_pos;
        param.sampling_rate = 125;
        param.type_signal = "eeg_controller";

        srv.call(param).then(() => {
            resolve();
        }).catch(() => {
            reject();
        });
    });
}

function load_dataset(user_name, id_experiment, num_ind, id_signal = "", storage_pos = -1) {
    return new Promise((resolve, reject) => {
        // carga de la señal
        let srv_lds = rosnodejs.nh.serviceClient("/load_dataset", "neurocontroller_database/load_dataset");
        let param = new neurocontroller_database.load_dataset.Request();
        param.user_name = user_name;
        param.id_experiment = id_experiment;
        param.num_ind = parseFloat(num_ind.split("_")[1]);
        param.id_eeg_signal = id_signal;
        param.storage_pos = storage_pos;

        srv_lds.call(param).then((res) => {
            resolve(res);
        }).catch((e) => {
            console.error("Error al cargar las señales\n", e);
            reject();
        });
    });
}


