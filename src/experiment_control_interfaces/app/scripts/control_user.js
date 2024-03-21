//----- devuelve el nombre de usuario -------
//--------------------------------------------
function get_actual_user() {
    let usr = document.getElementById("actual_user");

    let index = usr.selectedIndex;
    if (index < 0) return ""
    else return usr.options[index].value;
}

//carga los nombres de todos los usuarios registrados
function load_users() {
    return new Promise((resolve, reject) => {
        //se cargan los usuarios
        let sel_usr = document.getElementById("actual_user");

        fs.readdir(homedir + "/catkin_ws/src/neurocontroller_database/database", { withFileTypes: true }, (error, files) => {
            const directoriesInDIrectory = files.filter((item) => {
                return item.isDirectory();
            }).map((item) => item.name);

            sel_usr.innerHTML = "";
            for (let i = 0; i < directoriesInDIrectory.length; i++) {
                let usr_name = directoriesInDIrectory[i];
                if (usr_name != "optimized_individuals") {
                    let option = document.createElement("option");
                    option.setAttribute("value", usr_name);
                    option.innerHTML = usr_name;
                    sel_usr.appendChild(option);
                }
            }
            resolve(sel_usr);
        });
    });
}


//se crea un nuevo usuario
let user_txt = document.getElementById("new_user");
user_txt.addEventListener("keypress", (e, f) => {
    if (!(e.key == "Enter" || e.keyCode == 13)) return;

    let user_name = user_txt.value;
    if (user_name != undefined && user_name != "") {
        user_name = user_name.replace(/\s/g, "");
        let dir = homedir + "/catkin_ws/src/neurocontroller_database/database/" + user_name;

        //se crean la carpetas del nuevo usuario
        fs.mkdirSync(dir, { recursive: true });

        //se actualiza la lista de usuarios
        load_users().then((sel_usr) => {
            let size = sel_usr.options.length;
            let opt;
            for (let i = 0; i < size; i++) {
                let txt = sel_usr.options[i].value;

                if (txt == user_name) {
                    opt = sel_usr.options[i];
                    break;
                }
            }

            if (opt != undefined) {
                opt.selected = true;
                // se guarda
                change_param("user_name", user_name).catch((e) => {
                    console.error(e);
                    alert("Error al actualizar el usuario");
                });
            }
        });

        show_blink(user_txt, color_success);
    } else {
        show_blink(user_txt, color_error);
    }
});

// reset del experimento
function reset_experiment() {
    let user_name = get_actual_user();
    let dir_user = `${root_dir}/${user_name}`;
    let dir_histo = `${dir_user}/history`;

    // erase_file(`${dir_histo}/adquisition_data.out`);
    // erase_file(`${dir_histo}/emotional_roadmap.txt`);
    // erase_file(`${dir_histo}/front_direction.txt`);
    // erase_file(`${dir_histo}/selected_individuals.txt`);
    // erase_file(`${dir_histo}/prediction_data.txt`);
    // erase_file(`${dir_histo}/histo_rest.txt`);
    // // erase_file(`${dir_histo}/histo_basal_state.txt`);
    // erase_file(`${dir_user}/actual_front.txt`);
    // erase_file(`${dir_user}/actual_ind.txt`);
    // erase_file(`${dir_user}/end_experiment.txt`);
    // erase_file(`${dir_user}/point_reference.txt`);
    // erase_file(`${dir_user}/pos_fronts_table.txt`);

    erase_file(`${dir_user}/actual_ind.txt`);
    erase_file(`${dir_user}/customer_satisfied_score.txt`);
    erase_file(`${dir_user}/end_experiment.txt`);

    // se eliminan los historiales
    erase_directory(`${dir_user}/history`);

    // se eliminan los datos
    erase_directory(`${dir_user}/data_optimized_individuals`);
}

function erase_file(path) {
    try {
        fs.unlinkSync(path);
    } catch (err) {
        // console.error("Error al eliminar el archivo\n", err);

    }
}

function erase_directory(dir) {
    fs.rmdir(dir, { recursive: true }, err => {
        if (err) {
            // console.error("Error al eliminar el archivo", err);
            return;
        }
    });
}


//servicio para enviar el nombre de usuario
function ros_srv_get_user() {
    let srv = rosnodejs.nh.advertiseService("/get_user", "experiment_control_interfaces/get_user", (req, res) => {
        let usr = get_actual_user();

        if (usr == "") return false;

        res.user_name = usr;
        return true;
    });
}
ros_srv_get_user();




// ------------------------------------------------
// ------------------------------------------------

function experiment_neuro(control) {
    return new Promise((resolve, reject) => {
        // // estado basal 
        // let is_auto = document.getElementById("check_auto_inds").checked;

        // if (!is_auto) {
        //     // texto de los individuos
        //     let individuals_str = document.getElementById("individuals_str").value;
        //     // se convierte en array
        //     let individuals_array;
        //     try {
        //         individuals_array = JSON.parse(individuals_str)
        //     } catch (error) {
        //         alert("Error con el formato de los individuos");
        //         console.error(error);
        //         reject();
        //         return;
        //     }
        // } else {
        //     individuals_array = [];
        // }

        let individuals_array = [];

        // se carga el frente 
        let id_pareto = document.getElementById("id_pareto_load").value;
        id_pareto = id_pareto.trim();
        if (id_pareto.search("optimized-") < 0) {
            alert("Error con el formato del frente de pareto");
            reject();
            return;
        }

        // se carga el experimento
        let param = new user_interaction.experiment_control.Request();
        param.id_pareto_front = id_pareto;
        param.num_inds = individuals_array;
        param.control = control;
        param.play_video_intro = document.getElementById("check_intro_vid").checked;
        param.extend_search = document.getElementById("extend_search").checked;

        experiment_control(param).then(() => {
            resolve();
        }).catch((e) => {
            console.error(e);
            reject();
        });
    });
}


// inicio del experimento
let btn_start = document.getElementById("btn_start_experimento");
btn_start.onclick = () => {
    let type_load;
    if (document.getElementById("rad_is_start").checked) type_load = "start_experiment";
    // if (document.getElementById("rad_is_test_interface").checked) type_load = "test_interface";
    if (document.getElementById("rad_is_test").checked) type_load = "test_experiment";
    // if (document.getElementById("rad_is_protocol").checked) type_load = "start_protocol";

    // MODO SAM ACTIVADO
    if ((document.getElementById("rad_sam").checked || document.getElementById("rad_traditional").checked) && !document.getElementById("rad_is_protocol").checked) {
        experiment_neuro(type_load).then(() => {
            show_blink(btn_start, color_success);
        }).catch((e) => {
            show_blink(btn_start, color_error);
            console.error(e);
        });
        return;
    }

    // se verifica si hay tarjeta conectada
    get_state_acquisition().then((res_eeg) => {
        if (!res_eeg.connected) {
            alert("No hay tarjeta conectada");
            return;
        }

        if (document.getElementById("rad_is_protocol").checked) {
            // type_load == "start_protocol"
            start_protocol("start_protocol");
            return;
        }

        // se verifica si hay modelo cargado
        get_emotion_data().then((res_pred) => {
            if (!res_pred.is_loaded || res_pred.model_name == "") {
                alert("No hay modelo cargado");
                return;
            }

            experiment_neuro(type_load).then(() => {
                show_blink(btn_start, color_success);
            }).catch((e) => {
                show_blink(btn_start, color_error);
                console.error(e);
            });

        }).catch((e) => {
            alert("Error al iniciar el estado basal");
            console.error(e);
        });
    }).catch((e) => {
        alert("Error con la tarjeta");
        console.error(e);
    });
}

// cambio de usuario
let user_sel = document.getElementById("actual_user");
user_sel.addEventListener("change", () => {
    let user_name = get_selected(user_sel);

    change_param("user_name", user_name).catch((e) => {
        console.error(e);
        alert("Error al actualizar el usuario");
    });
});



// //--------------------------------------------
// //----- si el experimento extiende su búsqueda en cada frente visitado -------
// //--------------------------------------------

let param_extend = {}
param_extend["type"] = "checkbox";
param_extend["id"] = "extend_search";
param_extend["text"] = "<label>extender búsqueda</label>";
param_extend["non_br"] = true;
param_extend["checked_value"] = "extend_search";
param_extend["non_checked_value"] = "non_extend_search";
param_extend["ros_name"] = "/control_preference";
param_extend["ros_type"] = "user_interaction/control_preference";
param_extend["callback_prepare_param"] = function (input) {
    let param = new user_interaction.control_preference.Request();

    let ctrl = "non_extend_search";
    if (input.checked) ctrl = "extend_search";

    param.control = ctrl;

    return param;
};
let is_extend = create_input_param(param_extend);
is_extend.load_param();
document.getElementById("container_extend_search").appendChild(is_extend);

