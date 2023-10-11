

// ---------------------------------------------
// ---------------------------------------------
// num mensajes en la interfaz
// ---------------------------------------------
let param_visual = {}
param_visual["type"] = "number";
param_visual["id"] = "visual_num";
param_visual["text"] = "Número de mensajes en el EEG ";
let visual_num = create_input_param(param_visual);
visual_num.load_param();
document.getElementById("container_num_visual").appendChild(visual_num);




// ---------------------------------------------
// ---------------------------------------------
//  muestra la ventana de efectos visuales
// ---------------------------------------------

let btn_effect_win = document.getElementById("btn_effect_win");
btn_effect_win.onclick = () => {
    let param = new user_interaction.experiment_control.Request();
    param.start_effects_window = true;

    experiment_control(param);
}

// ---------------------------------------------
// ---------------------------------------------
// si es sel_automatic o sel_manual. Forma de seleccionar los individuos en la interfaz
// ---------------------------------------------

let container_v = document.getElementById("container_visual");

// si es manual la captura
let param_auto = {}
param_auto["type"] = "checkbox";
param_auto["id"] = "selection_mode";
param_auto["text"] = "<label class='name_control'>Seleccionar individuos automaticamente</label>";
param_auto["checked_value"] = "sel_automatic"
param_auto["non_checked_value"] = "sel_manual"
param_auto["ros_name"] = "/change_selection_ind";
param_auto["ros_type"] = "user_interaction/experiment_control";
param_auto["callback_prepare_param"] = function (input) {
    let param = new user_interaction.experiment_control.Request();
    let mode_sel = "sel_manual";
    if (input.checked) mode_sel = "sel_automatic";

    if (document.getElementById("rad_traditional").checked)
        document.getElementById("selection_mode").checked = false;

    param.control = mode_sel;
    return param;
};
let is_sel_auto = create_input_param(param_auto);
is_sel_auto.load_param();
container_v.appendChild(is_sel_auto);



// ---------------------------------------------
// ---------------------------------------------
// cambia el modo de trabajo 
// ---------------------------------------------
let container_mode = document.getElementById("container_mode");


function before_to_send() {
    is_sel_auto.ros_function();

    if (document.getElementById("rad_traditional").checked) {
        document.getElementById("selection_mode").checked = false;
    }
}


let param_mode = {};
param_mode["type"] = "radio";
param_mode["id"] = "rad_bci";
param_mode["value"] = "bci";
param_mode["text"] = "<label class='name_control'>Modo de trabajo</label><br>BCI";
param_mode["name"] = "emotions_capture_mode";
param_mode["ros_name"] = ["/change_mode_capture", "/control_emotions"];
param_mode["ros_type"] = ["user_interaction/experiment_control", "emotion_classification/control_emotions"];
param_mode["callback_prepare_param"] = [
    function (input) {
        before_to_send();
        let param = new user_interaction.experiment_control.Request();
        param.control = "bci";
        return param;
    },
    function (input) {
        let param = new emotion_classification.control_emotions.Request();
        param.control = "bci";
        return param;
    }
];
let mode_bci = create_input_param(param_mode);
mode_bci.load_param();
container_mode.appendChild(mode_bci);


param_mode = {};
param_mode["type"] = "radio";
param_mode["id"] = "rad_sam";
param_mode["text"] = "SAM";
param_mode["name"] = "emotions_capture_mode";
param_mode["value"] = "sam";
param_mode["ros_name"] = ["/change_mode_capture", "/control_emotions"];
param_mode["ros_type"] = ["user_interaction/experiment_control", "emotion_classification/control_emotions"];
param_mode["callback_prepare_param"] = [
    function (input) {
        before_to_send();
        let param = new user_interaction.experiment_control.Request();
        param.control = "sam";
        return param;
    },
    function (input) {
        let param = new emotion_classification.control_emotions.Request();
        param.control = "sam";
        return param;
    }
];
let mode_sam = create_input_param(param_mode);
mode_sam.load_param();
container_mode.appendChild(mode_sam);

param_mode = {};
param_mode["type"] = "radio";
param_mode["id"] = "rad_traditional";
param_mode["text"] = "tradicional";
param_mode["name"] = "emotions_capture_mode";
param_mode["value"] = "traditional";
param_mode["ros_name"] = ["/change_mode_capture", "/control_emotions"];
param_mode["ros_type"] = ["user_interaction/experiment_control", "emotion_classification/control_emotions"];
param_mode["callback_prepare_param"] = [
    function (input) {
        before_to_send();
        let param = new user_interaction.experiment_control.Request();
        param.control = "traditional";
        return param;
    },
    function (input) {
        let param = new emotion_classification.control_emotions.Request();
        param.control = "traditional";
        return param;
    }
];
let mode_tra = create_input_param(param_mode);
mode_tra.load_param();
container_mode.appendChild(mode_tra);


// -------------------------------------------------------
// -------------- tipo de gráfica ---------------------------------------
// -------------------------------------------------------

let param_graph = {};
param_graph["type"] = "radio";
param_graph["id"] = "rad_scatter";
param_graph["text"] = "<label class='name_control'>Gráfica para la interacción</label><br>scatter";
param_graph["name"] = "type_user_graph";
param_graph["value"] = "scatter";
param_graph["ros_name"] = "/change_type_graph";
param_graph["ros_type"] = "user_interaction/experiment_control";
param_graph["callback_prepare_param"] = function (input) {
    // before_to_send();
    let param = new user_interaction.experiment_control.Request();
    param.control = "scatter";
    return param;
};
let type_scatter = create_input_param(param_graph);
type_scatter.load_param();
container_mode.appendChild(type_scatter);


param_graph = {};
param_graph["type"] = "radio";
param_graph["id"] = "rad_parcoords";
param_graph["text"] = "parallel categories";
param_graph["name"] = "type_user_graph";
param_graph["value"] = "parcoords";
param_graph["ros_name"] = "/change_type_graph";
param_graph["ros_type"] = "user_interaction/experiment_control";
param_graph["callback_prepare_param"] = function (input) {
    // before_to_send();
    let param = new user_interaction.experiment_control.Request();
    param.control = "parcoords";
    return param;
};
let type_par_cat = create_input_param(param_graph);
type_par_cat.load_param();
container_mode.appendChild(type_par_cat);




// -------------------------------------------------------
// -------------- narmalización ---------------------------------------
// -------------------------------------------------------


// si es manual la captura
let param_norm = {}
param_norm["type"] = "checkbox";
param_norm["id"] = "normalization";
param_norm["text"] = "<label class='name_control'>Normalizar</label>";
param_norm["checked_value"] = "do_norm";
param_norm["non_checked_value"] = "deactiv_norm";
param_norm["ros_name"] = "/change_norm";
param_norm["ros_type"] = "user_interaction/experiment_control";
param_norm["callback_prepare_param"] = function (input) {
    let param = new user_interaction.experiment_control.Request();
    let mode_sel = "deactiv_norm";
    if (input.checked) mode_sel = "do_norm";

    param.control = mode_sel;
    console.log(param);
    return param;
};
let do_norm = create_input_param(param_norm);
do_norm.load_param();
container_v.appendChild(do_norm);



// document.getElementById("rad_bci").addEventListener("click", (rad) => {
//     change_param("emotions_capture_mode", "bci");
//     change_capture_mode("bci");
// });


// document.getElementById("rad_sam").addEventListener("click", (rad) => {
//     change_param("emotions_capture_mode", "sam");
//     change_capture_mode("sam");
// });



// function change_capture_mode(mode) {
//     return new Promise((resolve, reject) => {
//         if (mode != "bci" && mode != "sam") {
//             console.error(`Modo inexistente ${mode}`);
//             reject();
//             return;
//         }

//         // servicio para la interfaz gráfica
//         let srv = rosnodejs.nh.serviceClient("/change_mode_capture", "user_interaction/experiment_control");
//         let param = new user_interaction.experiment_control.Request();
//         param.control = mode;

//         srv.call(param).then(() => {
//             resolve();
//         }).catch(() => {
//             console.error("No se pudo cambiar el modo en la interfaz de usuario");
//         });

//         // servicio para el nodo de clasificación
//         srv = rosnodejs.nh.serviceClient("/change_mode_capture", "user_interaction/experiment_control");
//         param = new user_interaction.experiment_control.Request();
//         param.control = mode;

//         srv.call(param).then(() => {
//             resolve();
//         }).catch(() => {
//             console.error("No se pudo cambiar el modo en la interfaz de usuario");
//         });

//     });
// };

