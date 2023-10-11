//Nuevo modelo CNN, comportamiento local
let sel_new_model = document.getElementById("model_emotion");
let dr = create_field("dropout", "number");
dr[1].setAttribute("min", "0");
dr[1].setAttribute("max", "0");
document.getElementById("container_field_cnn").appendChild(dr[0]);

sel_new_model.addEventListener("change", (e) => {
    let index = sel_new_model.selectedIndex;
    let model_name = sel_new_model.options[index].value;

    let container = document.getElementById("container_field_cnn");
    container.innerHTML = "";

    let drp = create_field("dropout", "number");
    let k = create_field("kernLength", "number")
    let f1 = create_field("F1", "number");
    let d = create_field("D", "number");
    let f2 = create_field("F2", "number");

    container.appendChild(drp[0])
    if (model_name == "EEGNet") {
        container.appendChild(k[0]);
        container.appendChild(f1[0]);
        container.appendChild(d[0]);
        container.appendChild(f2[0]);
    }
});




//--------------------------------------------------------
//------------ crea un nuevo modelo emocional -----------------
create_input_ros("button", "Nuevo modelo", "/create_emotion_model", "emotion_classification/create_emotion_model",
    async (btn) => {
        let user_name = "diego";
        // await get_user_actual().then((usr) => {
        //     user_name = usr;
        // });

        let msg_create_m = new emotion_classification.create_emotion_model.Request();
        let sel = document.getElementById("model_emotion");
        let m_name = sel.options[sel.selectedIndex].value;

        msg_create_m.model_name = m_name;
        msg_create_m.user_name = user_name
        msg_create_m.dropout = parseFloat(document.getElementById("dropout").value);
        msg_create_m.num_channels = 16;
        msg_create_m.num_classes = 4;
        msg_create_m.num_samples = 128;

        if (m_name == "EEGNet") {
            msg_create_m.kernel_size = parseInt(document.getElementById("kernLength").value);
            msg_create_m.F1 = parseInt(document.getElementById("F1").value);
            msg_create_m.D = parseInt(document.getElementById("D").value);
            msg_create_m.F2 = parseInt(document.getElementById("F2").value);
            if (msg_create_m.kernel_size <= 0 &&
                msg_create_m.F1 <= 0 &&
                msg_create_m.D <= 0 &&
                msg_create_m.F2 <= 0) {
                alert("Los parámetros de la CNN deben ser mayores a cero")
                return;
            }
        }
        if (msg_create_m.dropout <= 0 || msg_create_m.dropout >= 1) {
            alert("Dropout: el valor debe estar entre (0,1)");
            return;
        }

        return msg_create_m;
    }, (res) => {
        console.log(res);
        update_input_load_emotion(res.id_emotion_model);
        show_blink(document.getElementById("container_new_emo"), color_success);
    }, "Crear un nuveo modelo para la detección de emociones").then((btn_new_model) => {
        document.getElementById("container_param_cnn").appendChild(btn_new_model);
    });


//nuevo campo 
function create_field(name, type_in) {
    let c = document.createElement("div");
    let lab = document.createElement("label");
    lab.innerHTML = name;
    let field = document.createElement("input");
    field.setAttribute("type", type_in);
    field.setAttribute("id", name);

    c.appendChild(lab);
    c.appendChild(field);

    return [c, field];
}


//---------------------------------------------------------
//---------------------------------------------------------
function update_select_idModel(model_name) {
    return new Promise((resolve, reject) => {
        // get_model_instancies(name).then((res) => {
        //     
        // });
        let user = get_selected(document.getElementById("actual_user"));
        let dir_model = `${root_dir}/${user}/emotion_model/${model_name}`;

        get_subdirs(dir_model).then((dirs) => {

            let sel_emo_id = document.createElement("select");

            //se agregan las nuevas opciones
            for (let i = 0; i < dirs.length; i++) {
                let opt = document.createElement("option");
                opt.setAttribute("value", dirs[i]);
                opt.innerHTML = dirs[i];

                sel_emo_id.appendChild(opt);
            }
            document.getElementById("emotion_model_id").remove();
            sel_emo_id.setAttribute("id", "emotion_model_id");
            document.getElementById("container_model_id").appendChild(sel_emo_id);
            resolve();
        });
    });
}



//-----------------------------------------------------
//---- botón para seleccionar el nombre del modelo ----

let sel_emo = document.getElementById("emotion_model_name");
sel_emo.addEventListener("change", () => {
    let name = sel_emo.options[sel_emo.selectedIndex].value;
    if (name == "") return;

    update_params_train({ model_name: name }).then(() => {
        show_blink(sel_emo, color_success);
        update_params_train({ model_name: name }).then(() => {
            // select del id
            update_select_idModel(name);

        });
    }).catch((e) => {
        console.error(e);
    });
});



//-----------------------------------------------------
//------------ botón para cargar el modelo ------------
let btn_load = document.getElementById("btn_load_model");
btn_load.addEventListener("click", () => {
    if (sel_emo.selectedIndex <= 0) {
        alert("Seleccionar el nombre del modelo emocional");
        return;
    }

    let id_name = document.getElementById("emotion_model_id");
    if (id_name.options.length == 0) {
        alert("Seleccionar ID de la intancia del modelo");
        return;
    }

    //llamos al servico ros para cargar el modelo
    let srv_model_emo = rosnodejs.nh.serviceClient("/load_emotion_model", "emotion_classification/load_emotion_model");
    let param = new emotion_classification.load_emotion_model.Request();
    param.user_name = get_actual_user();
    param.model_name = sel_emo.options[sel_emo.selectedIndex].value;
    param.id_emotion_model = id_name.options[id_name.selectedIndex].value;
    param.repetition_k = parseInt(document.getElementById("emotion_model_k").value);
    param.num_fold = parseInt(document.getElementById("emotion_model_fold").value);


    srv_model_emo.call(param).then((res) => {
        show_blink(document.getElementById("container_load_emo"), color_success);
    }).catch((e) => {
        console.error("Error al caragar el modelo emocional", e);
        show_blink(document.getElementById("container_load_emo"), color_error)
    });
});


//------------------------------------------------------
//-------------- actualiza los input del modelo --------
function update_input_load_emotion() {
    get_emotion_data().then((res) => {
        if (res.model_name == "" || res.id_emotion_model == "") return;
        console.log("update", res);
        let name = document.getElementById("emotion_model_name");
        let id = document.getElementById("emotion_model_id");

        let model_name = res.model_name;
        let id_model = res.id_emotion_model;

        //selector del modelo
        name.value = model_name;

        update_select_idModel(model_name).then((res2) => {
            id.value = id_model;
            document.getElementById("id_emotion_model").innerHTML = id_model;
        });
    });
}

//----------------------------------------------------------
//--------- Número de segundos a tamaño de ventana ---------
//----------------------------------------------------------


let win_stride = document.getElementById("stride");
let checkbox = document.getElementById("is_same_window");
let win_size = document.getElementById("in_window_size");
let batch_size_c = document.getElementById("batch_size");
let num_rep_c = document.getElementById("num_rep");
let num_fold_c = document.getElementById("num_fold");
let num_epoch_c = document.getElementById("num_epoch");
let tolerance_c = document.getElementById("tolerance");

// se carga el tamaño de ventana
document.body.onload = () => {
    // carga de los parámetros
    load_object(`${root_dir}/train_parameters.txt`).then((params) => {
        win_size.value = params.window_size;
        win_stride.value = params.stride;
        batch_size_c.value = params.batch_size;
        num_rep_c.value = params.k_repetitions;
        num_fold_c.value = params.size_fold;
        num_epoch_c.value = params.num_epoch;
        tolerance_c.value = params.tolerance;
    }).catch(() => {
        console.error("Error al cargar los parámetros de entrenamiento");
    })
};

function update_params_train(in_params) {
    return new Promise((resolve, reject) => {
        // model_name_, window_size, stride, num_rep, num_fold, num_epoch, tolerance, batch_size
        let ctl_emo = new emotion_classification.control_emotions.Request();

        // se cargan los paramteros
        load_object(`${root_dir}/train_parameters.txt`).then((params) => {
            params = { ...ctl_emo, ...params };
            if (in_params["model_name"] != undefined) params["model_name"] = in_params["model_name"];
            if (in_params["window_size"] != undefined) params["window_size"] = in_params["window_size"];
            if (in_params["stride"] != undefined) params["stride"] = in_params["stride"];
            if (in_params["num_rep"] != undefined) params["k_repetitions"] = in_params["num_rep"];
            if (in_params["num_fold"] != undefined) params["size_fold"] = in_params["num_fold"];
            if (in_params["num_epoch"] != undefined) params["num_epoch"] = in_params["num_epoch"];
            if (in_params["tolerance"] != undefined) params["tolerance"] = in_params["tolerance"];
            if (in_params["batch_size"] != undefined) params["batch_size"] = in_params["batch_size"];

            // se guardan los parámetros
            save_object(params, "train_parameters.txt", root_dir).then(() => {
                control_emotion(params).then(() => {
                    resolve();
                }).catch((e) => {
                    reject(e);
                });
            }).catch((e) => {
                console.error(e);
                reject();
            });
        });
    });
}


// evento de window_size
win_size.addEventListener("keypress", (e) => {
    if (e.key != "Enter") return;

    let win_s = parseInt(win_size.value);

    if (checkbox.checked) {
        win_stride.value = win_s;
        update_params_train({ stride: win_s }).then(() => {
            show_blink(win_stride, color_success);
        }).catch((e) => {
            console.error("Error al guardar el tamaño de ventana\n", e);
            show_blink(win_stride, color_error);
        });
    }

    // se guarda
    update_params_train({ window_size: win_s }).then(() => {
        show_blink(win_size, color_success);
    }).catch((e) => {
        console.error("Error al guardar el tamaño de ventana\n", e);
        show_blink(win_size, color_error);
    });
});

// evento del stride
win_stride.addEventListener("keypress", (e) => {
    if (e.key != "Enter") return;

    let win_s;
    if (checkbox.checked) {
        win_s = parseInt(win_size.value);
        win_stride.value = win_s;
    } else {
        win_s = parseInt(win_stride.value);
    }

    // se guarda
    update_params_train({ stride: win_s }).then(() => {
        show_blink(win_stride, color_success);
    }).catch(() => {
        console.error("Error guardar el tamaño del stride");
        show_blink(win_stride, color_error);
    });
});

// evento del checbox
checkbox.addEventListener("click", () => {
    if (checkbox.checked) {
        let w_s = parseInt(win_size.value);
        win_stride.value = w_s;

        update_params_train({ stride: w_s }).then(() => {
            show_blink(win_stride, color_success);
        }).catch(() => {
            console.error("Error al guardar el tamaño de ventana");
            show_blink(win_stride, color_error);
        });
    }
});

// -----------------------------------
// num rep
num_rep_c.addEventListener("keypress", (e) => {
    if (e.key != "Enter") return;

    let n_rep = parseInt(num_rep_c.value);

    update_params_train({ num_rep: n_rep }).then(() => {
        show_blink(num_rep_c, color_success);
    }).catch(() => {
        show_blink(num_rep_c, color_error);
    });
});

// -----------------------------------
num_fold_c.addEventListener("keypress", (e) => {
    if (e.key != "Enter") return;
    let n_f = parseInt(num_fold_c.value);

    update_params_train({ num_fold: n_f }).then(() => {
        show_blink(num_fold_c, color_success);
    }).catch(() => {
        show_blink(num_fold_c, color_error);
    });
});

// -----------------------------------------------
// -----------------------------------------------
tolerance_c.addEventListener("keypress", (e) => {
    if (e.key != "Enter") return;

    let t = parseInt(tolerance_c.value);

    update_params_train({ tolerance: t }).then(() => {
        show_blink(tolerance_c, color_success);
    }).catch(() => {
        show_blink(tolerance_c, color_error);
    });
});


// -----------------------------------------------
num_epoch_c.addEventListener("keypress", (e) => {
    if (e.key != "Enter") return;
    let n_e = parseInt(num_epoch_c.value);

    update_params_train({ num_epoch: n_e }).then(() => {
        show_blink(num_epoch_c, color_success);
    }).catch(() => {
        show_blink(num_epoch_c, color_error);
    });
});


//--------------------------------------------------
//-------------------- batch -----------------------
//--------------------------------------------------
batch_size_c.addEventListener("keypress", (e) => {
    if (e.key != "Enter") return;

    let val = parseInt(batch_size_c.value);

    update_params_train({ batch_size: val }).then(() => {
        show_blink(batch_size_c, color_success);
    }).catch(() => {
        show_blink(batch_size_c, color_error);
    });
});



// let btn_updateW = add_tooltip_and_blink(elemW, "Cambiar el tamaño del lote");
// document.getElementById("container_batch_size").appendChild(btn_updateW);


// function get_batch_size() {
//     let bz = document.getElementById("batch_size");
//     bz = parseInt(bz.value);
//     if (isNaN(bz)) {
//         console.error("Error al obtener el tamaño de lote");
//         return;
//     } else {
//         return bz;
//     }
// }


