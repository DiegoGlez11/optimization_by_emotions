
// parámetros de entrada de la optimización
let param_in = {}
param_in["type"] = "number";
param_in["id"] = "num_individuals_by_front";
param_in["text"] = "Número de individuos por frente";
param_in["ros_name"] = "/control_preference";
param_in["ros_type"] = "user_interaction/control_preference";
param_in["callback_prepare_param"] = function (input) {
    let param = new user_interaction.control_preference.Request();
    param.num_individuals_by_front = input.value;
    return param;
};
let num_ind_f = create_input_param(param_in);
num_ind_f.load_param();

param_in = {}
param_in["type"] = "number";
param_in["id"] = "search_threshold_front";
param_in["text"] = "Umbral de frentes de búsqueda";
param_in["ros_name"] = "/control_preference";
param_in["ros_type"] = "user_interaction/control_preference";
param_in["callback_prepare_param"] = function (input) {
    let param = new user_interaction.control_preference.Request();
    param.search_threshold_front = input.value;
    return param;
};
let threshold_f = create_input_param(param_in);
threshold_f.load_param();

let container_opt = document.getElementById("container_optimization");
container_opt.appendChild(num_ind_f);
container_opt.appendChild(threshold_f);


// //--------------------------------------------
// //----- número de frentes a cargar a partir de la pos del frente actual  -------
// //--------------------------------------------

param_in = {}
param_in["type"] = "number";
param_in["id"] = "size_loaded_fronts";
param_in["text"] = "Número de frentes a<br>cargar a partir del actual";
param_in["ros_name"] = "/control_preference";
param_in["ros_type"] = "user_interaction/control_preference";
param_in["callback_prepare_param"] = function (input) {
    let param = new user_interaction.control_preference.Request();
    param.size_loaded_fronts = input.value;
    return param;
};
let num_fronts = create_input_param(param_in);
num_fronts.load_param();

container_opt.appendChild(num_fronts);









// //--------------------------------------------
// //----- botón de envio de preferencias -------
// //--------------------------------------------
// create_input_ros("button", "Enviar", "/preference_optimizer", "nsga2_dmaking/local_search",
//     (btn) => {//función que retorna el mensaje de solicitud de un servico ROS
//         let x = parseFloat(document.getElementById("x").value);
//         let y = parseFloat(document.getElementById("y").value);
//         let actual_phylo = get_actual_phylo();

//         if (isNaN(x) || isNaN(y)) return;
//         if (!exist_user()) return;
//         if (actual_phylo == undefined) return;

//         //----------------------------------------------------------
//         //ID de la nueva población
//         let id = actual_phylo.id + "_" + actual_phylo.childs.length;
//         let msg = { point: [x, y], id_pareto_front: id, user_name: get_actual_user() };

//         return msg;
//     },
//     (res_) => {//acción a realizar tras la respuesta del servicio
//         if (!exist_user()) return;

//         //se calcula el ID de la población (frente de pareto) padre
//         let code_id = res_.id_pareto_front.split("_");
//         code_id.pop();
//         let id_parent = code_id.join("_");

//         //carga de la info filogenetica del padre
//         load_phylogenetic_data(id_parent).then((parent_node) => {
//             //se crea la info del hijo
//             let node_data = Object.assign({}, population);
//             node_data.parent_id = id_parent;
//             node_data.id = res_.id_pareto_front;
//             node_data.n_gen = res_.n_gen;

//             //se agrega al árbol filogenético
//             node_data.num_child = parent_node.childs.length;
//             parent_node.childs.push(node_data.id);
//             node_data.childs = [];
//             //posición en el dataset de la gráfica
//             node_data.pos_database = globalChartRef.data.datasets.length;


//             //se guarda la info filogenetica del hijo
//             save_phylogenetic_data(node_data.id, node_data).then(() => {
//                 //se actualiza la info del padre
//                 save_phylogenetic_data(parent_node.id, parent_node).then(() => {
//                     //Se agregan los objetivos del nuevo frente a la gráfica
//                     update_fromArray1D(res_.obj_space, node_data, save = true).then(() => {
//                         select_pop(node_data);
//                     });
//                 });
//             })

//         });

//     }, "Enviar el punto de referencia al optimizador").then((btn_send) => {
//         let btn_cont = document.getElementById("btn_send_preference");
//         btn_cont.style.display = "flex";
//         btn_cont.appendChild(btn_send);
//     });




// //------- boton para iniciar una nueva población -----
// //-----------------------------------------------

// create_input_ros("number", "Población inicial", "/new_population_nsga2", "nsga2_dmaking/new_population_nsga2",
//     async function (input) {
//         //manejo de errores
//         if (input.value == "" || input.value == undefined) {
//             show_blink(input, color_error);
//             alert("Ingresar el número de individuos en la población inicial");
//             return;
//         }
//         if (input.value % 4 > 0) {
//             show_blink(input, color_error);
//             alert("El número de individuos debe ser divisible entre 4");
//             return;
//         }
//         if (!exist_user()) return;

//         //se calcula el id de la nueva población
//         let dir_root = [];
//         await new Promise((resolve, reject) => {
//             let dir_front = homedir + "/catkin_ws/src/neurocontroller_database/database/" + get_actual_user() + "/pareto_fronts";
//             fs.readdir(dir_front, { withFileTypes: true }, (error, files) => {
//                 if (error) reject();

//                 const directoriesInDIrectory = files.filter((item) => {
//                     //se buscan todas las poblaciones root
//                     if (item.name.search("root") > -1) {
//                         return item.isDirectory();
//                     }
//                 }).map((item) => item.name);

//                 dir_root = directoriesInDIrectory;
//                 resolve();
//             });
//         });

//         //calculamos el valor máximo de root
//         let max;
//         for (let i = 0; i < dir_root.length; i++) {
//             let s = dir_root[i].split("-");
//             if (s.length == 2) {
//                 let n = parseInt(s[1]);
//                 if (!isNaN(n)) {
//                     if (max == undefined) max = n;
//                     else {
//                         if (max < n) max = n;
//                     }
//                 }
//             }
//         }

//         let id_p = "";
//         if (dir_root.length == 0) {
//             id_p = "_root-0";
//         } else {
//             id_p = "_root-" + (max + 1);
//         }

//         let msg = Object.assign({}, srv_control_optimization);
//         msg.action = "init";
//         msg.pop_size = parseInt(input.value);
//         msg.id_pareto_front = id_p;
//         msg.user_name = get_actual_user();
//         return msg;
//     }, (res) => {//la respuesta del servicio regresa el espacio de los objetivos
//         let num_obj = res.num_obj;
//         let obj_space = res.obj_space;

//         if (!exist_user()) return;

//         //se crea la info filogenetica de la nueva población
//         let node_data = Object.assign({}, population);
//         node_data.n_gen = res.n_gen;
//         node_data.id = res.id_pareto_front;
//         node_data.num_child = 0;
//         node_data.childs = [];
//         node_data.pos_database = globalChartRef.data.datasets.length;

//         //se guarda la info
//         save_phylogenetic_data(node_data.id, node_data).then(() => {
//             update_fromArray1D(obj_space, node_data, save = true).then(() => {
//                 select_pop(node_data);
//             });
//         });

//     }, "Tamaño de la nueva población de neurocontroladores aleatoriamente").then((btn_ini) => {
//         document.getElementById("init_population").appendChild(btn_ini);
//     });




// //-----------------------------------------------------------------
// //-----------------------------------------------------------------
// //-----------------------------------------------------------------
// create_input_ros("number", "Probabilidad de entrecruzamiento", "/set_optimization_param", "nsga2_dmaking/set_optimization_param",
//     (btn) => {
//         let val = parseFloat(btn.value);
//         if (val > 0 && !isNaN(val)) {
//             let msg = Object.assign({}, srv_set_optimization_param);
//             msg.param = "p_cross_real";
//             msg.val = val;
//             return msg;
//         }
//         return;
//     }, (res) => {
//         show_blink(document.getElementById("Probabilidaddeentrecruzamiento"), color_success);
//     }, "valores entre 0 y 1").then((elem) => {
//         document.getElementById("container_optimization").appendChild(elem);
//     }).catch((err) => {
//         console.error(err);
//         show_blink(document.getElementById("Probabilidaddeentrecruzamiento"), color_error);
//     });

// //-----------------------------------------------------------------
// //-----------------------------------------------------------------
// //-----------------------------------------------------------------
// create_input_ros("number", "eta mutación", "/set_optimization_param", "nsga2_dmaking/set_optimization_param",
//     (btn) => {
//         let val = parseFloat(btn.value);
//         if (val > 0 && !isNaN(val)) {
//             let msg = Object.assign({}, srv_set_optimization_param);
//             msg.param = "eta_m";
//             msg.val = val;
//             return msg;
//         }
//         return;
//     },
//     (res) => {
//         if (res.error == "") {
//             show_blink(document.getElementById("etamutación"), color_success);
//         } else {
//             console.error(res.error);
//             show_blink(document.getElementById("etamutación"), color_error);
//         }
//     }, "valores entre 0 y 1").then((elem) => {
//         document.getElementById("container_optimization").appendChild(elem);
//     });

// //-----------------------------------------------------------------
// //-----------------------------------------------------------------
// //-----------------------------------------------------------------
// create_input_ros("number", "eta entrecruzamiento", "/set_optimization_param", "nsga2_dmaking/set_optimization_param",
//     (btn) => {
//         let val = parseFloat(btn.value);
//         if (val > 0 && !isNaN(val)) {
//             let msg = Object.assign({}, srv_set_optimization_param);
//             msg.param = "eta_c";
//             msg.val = val;
//             return msg;
//         }
//         return;
//     },
//     (res) => {
//         if (res.error == "") {
//             show_blink(document.getElementById("etaentrecruzamiento"), color_success);
//         } else {
//             console.error(res.error);
//             show_blink(document.getElementById("etaentrecruzamiento"), color_error);
//         }
//     }, "valores entre 0 y 1").then((elem) => {
//         document.getElementById("container_optimization").appendChild(elem);
//     });



// //-----------------------------------------------------------------
// //-----------------------------------------------------------------
// //-----------------------------------------------------------------
// async function get_params_optimizer() {
//     let srv = rosnodejs.nh.serviceClient("/get_optimization_param", "nsga2_dmaking/get_optimization_param");
//     srv.call({}).then((res) => {
//         //proba de mutación
//         let p_mut = document.getElementById("p_mut");
//         p_mut.innerHTML = res.p_mut_real;

//         //proba de entrecruzamiento
//         let p_cross = document.getElementById("Probabilidaddeentrecruzamiento");
//         p_cross.setAttribute("value", res.p_cross_real);

//         //eta m
//         let eta_m = document.getElementById("etamutación");
//         eta_m.setAttribute("value", res.eta_m);

//         //eta entrecruzamiento
//         let eta_c = document.getElementById("etaentrecruzamiento");
//         eta_c.setAttribute("value", res.eta_c);


//         document.getElementById("n_gen").innerHTML = res.n_gen;
//         let id = "";
//         if (res.id_pareto_front == "") id = ". ";
//         else id = res.id_pareto_front;
//         document.getElementById("id_pareto_front").innerHTML = id;
//         document.getElementById("pop_size").innerHTML = res.pop_size;
//         document.getElementById("num_var").innerHTML = res.num_var;
//     });
// }

// get_params_optimizer().catch((err) => {
//     console.error(err);
// });





