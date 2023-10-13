//----------------------------------------
//--------- crear input con tooltip -----------------
//-----------------------------------------

// function add_tooltip_and_blink(elem, tooltip_msg) {
//   elem.setAttribute("class", "blink_show");
//   let btn_update = add_tooltip(elem, tooltip_msg);
//   let c1 = document.createElement("div");
//   c1.setAttribute("class", "blink");
//   c1.appendChild(btn_update);
//   let c2 = document.createElement("div");
//   c2.appendChild(c1);

//   return c2;
// }

// function add_tooltip(elem, tooltip_msg) {
//   let tooltip = document.createElement("a");
//   tooltip.setAttribute("alt", tooltip_msg);
//   tooltip.setAttribute("class", "tooltip");

//   tooltip.appendChild(elem);
//   return tooltip;
// }

//----------------------------------------
//--------- crear input ROS -----------------
//-----------------------------------------
// async function create_input_ros(type_input, name, service, msg_type, callback_prepare_msg, callback_ros = undefined, tooltip_txt = "") {
//   let elem = document.createElement("div");
//   let name_clean = name.replace(/\s/g, '');
//   let lab = document.createElement("label");
//   lab.setAttribute("for", name_clean);
//   lab.innerHTML = name;
//   let event_ = undefined;

//   if (type_input == "button2") {
//     input_change = document.createElement("button");
//     event_ = "click";
//   }
//   if (type_input == "select") {
//     let models = ["EEGNet", "DeepConvNet", "ShallowConvNet", "DeepForest"]
//     input_change = create_select(models);
//     event_ = "change";
//   } else {
//     input_change = document.createElement("input");
//     input_change.setAttribute("type", type_input);
//   }
//   if (type_input != "button" && type_input != "button2") {
//     elem.appendChild(lab);
//   }
//   //tooltip
//   let in_elem = input_change;
//   input_change = add_tooltip(input_change, tooltip_txt);

//   in_elem.setAttribute("class", "blink_show");
//   let blink = document.createElement("div");
//   blink.setAttribute("class", "blink");
//   blink.appendChild(input_change);
//   input_change = blink;

//   if (type_input == "button") {
//     event_ = "click";
//     in_elem.setAttribute("value", name);
//   }

//   if (type_input == "checkbox") {
//     event_ = "click";
//   }
//   if (type_input == "number" || type_input == "text" || type_input == "file") {
//     in_elem.setAttribute("id", name_clean);
//     event_ = "keypress";
//   }

//   //se agrega el evento al input
//   input_change.addEventListener(event_, async (e) => {
//     if (type_input == "number" || type_input == "text") {
//       if (!(e.key == "Enter" || e.keyCode == 13)) return;
//     }

//     await new Promise((res, rej) => {
//       let msg = callback_prepare_msg(in_elem);
//       res(msg);
//     }).then((msg) => {
//       if (msg == undefined) return;
//       console.log("mensaje enviado", msg);
//       console.log(service, msg_type);
//       //conexión con el servicio ROS
//       let srv_ = rosnodejs.nh.serviceClient(service, msg_type);
//       srv_.call(msg).then((res) => {
//         show_blink(blink, color_success);
//         if (callback_ros != undefined) callback_ros(res);
//       }).catch((e) => {
//         show_blink(blink, color_error);
//         console.log("Error con el servicio: ", service, " type: ", msg_type, "\n", e);
//       });
//     });
//   });

//   elem.appendChild(input_change);

//   //funcion para enviar un dato a la red ros
//   elem["ros_send_data"] = function (msg) {
//     //conexión con el servicio ROS
//     let ros_srv = rosnodejs.nh.serviceClient(service, msg_type);
//     ros_srv.call(msg).then((res) => {
//       if (callback_ros != undefined) callback_ros(res);
//       show_blink(blink, color_success);
//     }).catch((e) => {
//       show_blink(blink, color_error);
//       console.log("Error con el servicio: ", service, " type: ", msg_type, "\n", e);
//     });
//   };
//   //devuelve el input element
//   elem["get_input"] = () => {
//     return in_elem;
//   };

//   //guarda el tipo de evento
//   elem["type_event"] = event_;
//   return elem;
// }


// function create_select(options, evals = undefined) {
//   let input_change = document.createElement("select");
//   for (let i = 0; i < options.length; i++) {
//     let opt = document.createElement("option");
//     opt.setAttribute("value", options[i].replace(/\s/g, '_'));
//     opt.innerHTML = options[i];
//     input_change.appendChild(opt);

//     // if (evals != undefined) {
//     //   console.error("in eval create");
//     //   opt.addEventListener("click", () => {
//     //     console.log(options[i]);
//     //     console.log(evals[options[i]]);
//     //   });
//     // }
//   }
//   return input_change;
// }







function show_blink(elem, color) {
  // elem.style.visibility = "visible";
  elem.style.backgroundColor = color;
  setTimeout(() => {
    elem.style.backgroundColor = "transparent";
    // elem.style.visibility = "hidden";
  }, 700);
}

function load_object(path_obj) {
  return new Promise((resolve, reject) => {
    fs.readFile(path_obj, 'utf8', (err, data) => {
      if (err) {
        resolve({});
      } else {
        let obj = {};
        try {
          if (data.length > 0) obj = JSON.parse(data);
          resolve(obj);
        } catch (error) {
          reject(`Error al cargar: ${path_obj}`)
        }
      }
    });
  });
}


function save_object(obj, name_file, dir_) {
  return new Promise((res, rej) => {
    save_data_string(JSON.stringify(obj), name_file, dir_).then((e) => {
      res(e);
    }).catch((g) => {
      rej(g);
    });
  });
}



/* View in fullscreen */
function openFullscreen() {
  let elem = document.documentElement;

  if (elem.requestFullscreen) {
    elem.requestFullscreen();
  } else if (elem.webkitRequestFullscreen) { /* Safari */
    elem.webkitRequestFullscreen();
  } else if (elem.msRequestFullscreen) { /* IE11 */
    elem.msRequestFullscreen();
  }
}

/* Close fullscreen */
function closeFullscreen() {
  if (document.exitFullscreen) {
    document.exitFullscreen();
  } else if (document.webkitExitFullscreen) { /* Safari */
    document.webkitExitFullscreen();
  } else if (document.msExitFullscreen) { /* IE11 */
    document.msExitFullscreen();
  }
}

function get_base_dataset(name, type_g = undefined) {

  let t_g = "scatter";
  let mode = 'markers';
  if (type_g != undefined) t_g = type_g;

  let data_emo = {
    x: [],
    y: [],
    mode: mode,
    type: t_g,
    name: name,
    text: [],
    showlegend: true,
    marker: {
      // color: [],
      size: 3,
      opacity: 1,
      symbol: [],
      line: {
        width: 1,
        // color: [],
      },
    },
  };

  if (t_g == "parcoords") {
    let dimensions = [];
    let pos = get_pos_objectives(index_objectives, TYPE_GRAPH);
    for (let i = 0; i < pos.names.length; i++) {
      dimensions.push({ label: pos.names[i], values: [0], range: [0, 1] })
    }
    data_emo.dimensions = dimensions;
    data_emo["labelfont"] = { size: 18 };
    data_emo["tickfont"] = { size: 13 };
    data_emo["unselected"] = { line: { color: "rgb(10,136,186)", } };
    data_emo["line"] = { showscale: false, colorscale: 'Portland', cmin: 0, cmax: 1, color: [], }

  }
  if (t_g == "scatter" && name == "") {
    data_emo.x = [0];
    data_emo.y = [0];
  }

  return data_emo;
}


function map_val(val, ini_origin_range, end_origin_range, ini_new_range, end_new_range) {

  let dif_new = end_new_range - ini_new_range;

  //tamaño rango inicial
  let dif_origin = end_origin_range - ini_origin_range;
  //posición del valor en el rango
  let pos_origin = val - ini_origin_range;

  //mapeo
  let conv = (pos_origin * dif_new) / dif_origin;
  conv += ini_new_range;
  return conv;
}

function constrain(val, lim_inf, lim_sup) {
  if (val < lim_inf) return lim_inf;
  if (val > lim_sup) return lim_sup;
  return val;
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// mantiene la ventana a un tamaño establecido según el experimento
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


function resize_win(type_resize) {
  let w, h;
  if (type_resize == "traditional" || type_resize == "full") {
    w = window.screen.width;
    h = window.screen.height;
  }

  if (type_resize == "sam" || type_resize == "bci" || type_resize == "normal") {
    w = 919;
    h = 1020;
  }

  if (w != undefined && h != undefined)
    ipcRenderer.send("resize-me-please", { width: w, height: h })
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++ control de visibilidad de las ventanas en la interfaz ++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

function show_one_window(name) {

  let window = {};
  let window_name = [];

  window["graph"] = document.getElementById("container_graph");
  if (name != "graph") window_name.push("graph");

  window["fatigue"] = document.getElementById("container_fatigue");
  if (name != "fatigue") window_name.push("fatigue");

  window["intro_video"] = document.getElementById("container_intro_video");
  if (name != "intro_video") window_name.push("intro_video");

  window["questionnaire"] = document.getElementById("container_questionnaire");
  if (name != "questionnaire") window_name.push("questionnaire");

  window["goodbye"] = document.getElementById("end_experiment");
  if (name != "goodbye") window_name.push("goodbye");

  window["lim_neuro"] = document.getElementById("window_last_front");
  if (name != "lim_neuro") window_name.push("lim_neuro");

  window["msg_all_window"] = document.getElementById("msg_all_window");
  if (name != "msg_all_window") window_name.push("msg_all_window");

  // se ocultan las ventanas
  for (let i = 0; i < window_name.length; i++) {
    window[window_name[i]].style.display = "none";
  }
  //se muestra la ventana
  window[name].style.display = "";
}


function hide_all_windows() {
  document.getElementById("container_graph").style.display = "none";
  document.getElementById("container_fatigue").style.display = "none";
  document.getElementById("container_intro_video").style.display = "none";
  document.getElementById("container_questionnaire").style.display = "none";
  document.getElementById("end_experiment").style.display = "none";
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
function is_conn_refused(e) {
  return e.code.search("CONNREFUSED") >= 0;
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++  cargar/actualizar del ind actual ++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

function update_actual_ind(id_pareto, num_ind, storage_pos) {
  return new Promise((resolve, reject) => {

    get_dir().then((user_dir) => {

      //se carga la info del individuo actual
      load_object(`${user_dir}/actual_ind.txt`).then((actual_ind) => {
        actual_ind["id_experiment"] = id_pareto;
        actual_ind["num_ind"] = num_ind;
        actual_ind["storage_pos"] = storage_pos;

        save_object(actual_ind, "actual_ind.txt", user_dir).then(() => {
          resolve();
        }).catch((e) => {
          console.error(`Error al actualizar el individuo actual: ${id_pareto} ${num_ind} ${storage_pos}`);
          reject(e);
        });
      });
    }).catch((e) => {
      console.error(`Error al actualizar el individuo actual: ${id_pareto} ${num_ind} ${storage_pos}`);
      reject(e);
    });
  });
}

function load_actual_ind() {
  return new Promise((resolve, reject) => {
    get_dir().then((user_dir) => {
      load_object(`${user_dir}/actual_ind.txt`).then((actual_ind) => {
        resolve(actual_ind);
      });
    }).catch((e) => {
      console.error(`Error al cargar el individuo actual`);
      reject(e);
    });
  });
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++  cargar/actualizar el roadmap +++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

function add_to_roadmap(id_pareto, num_ind, storage_pos) {
  return new Promise((resolve, reject) => {
    get_dir().then((user_dir) => {
      let dir_histo = `${user_dir}/history`;

      //se actualiza el historial de los individuos seleccionados
      load_object(`${dir_histo}/emotional_roadmap.txt`).then((histo_ind) => {
        if (!Array.isArray(histo_ind)) histo_ind = [];

        //nuevo registro en el historial
        histo_ind.push({ id_pareto_front: id_pareto, num_ind: num_ind, storage_position: String(storage_pos) });

        save_object(histo_ind, "emotional_roadmap.txt", dir_histo).then(() => {
          resolve();
        }).catch((e) => {
          console.error(`Error al actualizar el roadmap: ${id_pareto} ${num_ind} ${storage_pos}`);
          reject(e);
        });
      });
    }).catch((e) => {
      console.error(`Error al actualizar el roadmap: ${id_pareto} ${num_ind} ${storage_pos}`);
      reject(e);
    });
  });
}


function load_roadmap() {
  return new Promise((resolve, reject) => {

    get_dir().then((user_dir) => {
      let dir_histo = `${user_dir}/history`;

      //se actualiza el historial de los individuos seleccionados
      load_object(`${dir_histo}/emotional_roadmap.txt`).then((histo_ind) => {
        resolve(histo_ind);
      });
    }).catch((e) => {
      console.error(`Error al cargar el roadmap`);
      reject(e);
    });

  });
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++  cargar/actualizar los experimentos ++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

function new_experiment(test_interface = undefined) {
  return new Promise((resolve, reject) => {

    get_dir().then((user_dir) => {
      let dir_hist = `${user_dir}/history`;

      // registro de experimentos
      load_object(`${dir_hist}/experiment_record.txt`).then((exp_reg) => {
        if (!Array.isArray(exp_reg["list"])) exp_reg["list"] = [];

        // nueva ID del experimento
        let storage_pos = exp_reg["list"].length;

        // se registra en la lista
        exp_reg["list"].push({ type: EMOTIONS_CAPTURE_MODE, test_interface: test_interface });

        // se registra por id
        if (exp_reg["dict"] == undefined) exp_reg["dict"] = {};
        exp_reg["dict"][String(storage_pos)] = storage_pos;

        save_object(exp_reg, "experiment_record.txt", dir_hist).then(() => {
          resolve(String(storage_pos));
        }).catch((e) => {
          console.log(`Error al crear un nuevo experimento`);
          reject(e);
        });
      });
    }).catch((e) => {
      console.log(`Error al crear un nuevo experimento`);
      reject(e);
    });
  });
}


function update_experiment(obj_exp, storage_pos) {
  return new Promise((resolve, reject) => {

    get_dir().then((user_dir) => {
      let histo_dir = `${user_dir}/history`;

      //se actualiza el historial de experimentos
      load_object(`${histo_dir}/experiment_record.txt`).then((reg_exp) => {
        let pos = reg_exp["dict"][storage_pos];
        let data_emo = reg_exp["list"][pos];

        if (data_emo == undefined || storage_pos == undefined) {
          reject(`No se encuentra el experimento: ${storage_pos}`);
          return;
        }

        // se concatena los datos del experimento
        data_emo = { ...data_emo, ...obj_exp };
        reg_exp["list"][pos] = data_emo;

        // se actualiza el experimento
        save_object(reg_exp, "experiment_record.txt", histo_dir).then(() => {
          resolve();
        }).catch((e) => {
          console.error(e);
        });
      });
    }).catch((e) => {
      reject(e);
    });

  });
}


function get_concluded_experiments() {
  return new Promise((resolve, reject) => {
    get_dir().then((user_dir) => {
      let histo_dir = `${user_dir}/history`;

      //se carga el historial de experimentos
      load_object(`${histo_dir}/experiment_record.txt`).then((reg_exp) => {
        let sam_list = [];
        let trad_list = [];
        let bci_list = [];

        // se recorre la lista de experimentos
        let exp_list = reg_exp["list"];
        for (let i = 0; i < exp_list.length; i++) {

          // si es prueba de interfaz
          if (exp_list[i]["test_interface"]) continue;

          // tipo de terminación del experimento
          if (exp_list[i]["type_end"] != undefined) {
            let type_exp = exp_list[i]["type"];

            if (type_exp == "sam") sam_list.push(exp_list[i]);
            if (type_exp == "traditional") trad_list.push(exp_list[i]);
            if (type_exp == "bci") bci_list.push(exp_list[i]);
          }
        }

        resolve({ sam: sam_list, traditional: trad_list, bci: bci_list });

      });
    }).catch((e) => {
      reject(e);
    });
  });
}

// ++++++++++++++++++++++++++++++++++++++++++++
// +++++ reseteo de la interfaz +++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++

function reset_interface() {
  return new Promise((resolve, reject) => {

    // se resetea el ind actual
    update_actual_ind("", "", "").then(() => {

      // se ocultan las partes gráficas
      document.getElementById("right_metric").style.visibility = "hidden";
      document.getElementById("left_metric").style.visibility = "hidden";
      document.getElementById("window_block").style.visibility = "hidden";
      document.getElementById("window_new_fronts").style.visibility = "hidden";
      // document.getElementById("window_last_front").style.visibility = "hidden";

      // si existe la tabla de objetivos
      let table = document.getElementById("inds_table");
      if (table != undefined) {
        let aux = table.childNodes[0];
        table.innerHTML = "";
        table.appendChild(aux);
      }

      // reseteo de los frentes
      ds_fronts = {};
      pos_ind = {};
      // frentes en fila
      DATA_FRONT = [];
      // control del flujo
      DATA_CONTROL.actual_front = "";
      // DATA_CONTROL.select_ind = false;
      is_select_ind(false);

      // gráfica inicial
      Plotly.react(GRAPH_OBJ_SPACE, [get_base_dataset("", TYPE_GRAPH)], get_layout(TYPE_GRAPH), { displaylogo: false, modeBarButtonsToRemove: ["zoom2d", 'lasso2d', "select2d", "toImage", "autoScale2d"] });

      resolve();
    }).catch((e) => {
      reject(e);
    });
  });

}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++  cambio de modo ++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

function change_mode_interfaz(mode) {

  if (mode == "traditional" || mode == "sam" || mode == "bci") {
    EMOTIONS_CAPTURE_MODE = mode;

    let msg_tab = document.getElementById("msg_table_obj");
    let btn_next = document.getElementById("next_individual");

    if (mode == "traditional") {
      // document.getElementById("next_individual").style.visibility = "hidden";
      document.getElementById("container_timer").style.visibility = "hidden";
      if (btn_next != undefined) btn_next.style = "hidden";
      if (msg_tab != undefined) msg_tab.innerHTML = "hidden";

      SELECTION_MODE = "sel_manual";

      // texto
      if (msg_tab != undefined) msg_tab.innerHTML = "Click en la relación de tu preferencia";

      // tamaño de la ventana igual a las dimensiones de la pantalla (screen)
      resize_win("traditional");
    } else {
      // document.getElementById("next_individual").style.visibility = "";
      if (btn_next != undefined) btn_next.style = "";
      if (msg_tab != undefined) msg_tab.innerHTML = "";
    }



    if (mode == "sam" || mode == "bci") {
      // document.getElementById("next_individual").style.visibility = "";

      document.getElementById("container_timer").style.visibility = "";
      resize_win("sam")
    }

    reset_interface();

  } else {
    alert(`No existe el modo de trabajo para la interfaz: ${mode}`);
  }
}

function change_mode_selection(mode) {
  if (mode == "sel_manual" || mode == "sel_automatic") {

    let btn_next = document.getElementById("next_individual");

    if (EMOTIONS_CAPTURE_MODE == "traditional") {
      document.getElementById("container_timer").style.visibility = "hidden";
      // document.getElementById("next_individual").style.visibility = "hidden";
      if (btn_next != undefined) btn_next.style.visibility = "hidden";
      SELECTION_MODE = "sel_manual";

    } else if (EMOTIONS_CAPTURE_MODE == "sam" || EMOTIONS_CAPTURE_MODE == "bci") {
      document.getElementById("container_timer").style.visibility = "";
      SELECTION_MODE = mode;

      if (SELECTION_MODE == "sel_automatic") {
        document.getElementById("next_individual").style.visibility = "hidden";

        if (DATA_FRONT.length >= 0 && !DATA_CONTROL.select_ind) {
          // console.log("event front from change sel node");
          send_data_front();
        }
      } else {
        // document.getElementById("next_individual").style.visibility = "";
        if (btn_next != undefined) btn_next.style.visibility = "";
      }


    } else {
      alert(`No existe el modo de trabajo para la interfaz: ${mode}`);
      return;
    }
  } else {
    alert(`No existe el modo de selección`);
  }
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++  cambio de la gráfica en la interfaz ++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

function change_graph(type_g) {
  if (type_g == "scatter" || type_g == "parcoords") {
    TYPE_GRAPH = type_g;

    if (DATA_CONTROL.select_ind) {
      console.error("Se esta ejecutando un individuo. No se puede cambiar la gráfica");
      return false;
    }


    if (TYPE_GRAPH == "parcoords") {
      // contenedor de la tabla
      let c = document.createElement("div");
      c.setAttribute("id", "container_table");

      // se crea la tabla
      let pos_obj = get_pos_objectives(index_objectives);
      let t = create_table_obj(pos_obj.names);
      c.appendChild(t);
      // console.log(EMOTIONS_CAPTURE_MODE == "traditional", "traditional", EMOTIONS_CAPTURE_MODE);
      // if (EMOTIONS_CAPTURE_MODE == "traditional") {
      //   let txt = document.createElement("div");
      //   txt.innerHTML = "Click en la relación de tu preferencia";
      //   c.appendChild(txt);
      // }

      // contenedor de la gráfica y la tabla
      let cont = document.getElementById("container_graph_obj_space"); // container_graph_obj_space , container_object_spce
      cont.innerHTML = "";
      cont.appendChild(c);
      cont.appendChild(GRAPH_OBJ_SPACE);

      // cont.insertAdjacentElement("afterbegin", c);
      // si ya existe la tabla
      // let t_exist = document.getElementById("inds_table");
      // let t_exist = document.getElementById("container_table");
      // if (t_exist == undefined) {
      // }

    } else {
      let msg_tab = document.getElementById("msg_table_obj");
      if (msg_tab != undefined) msg_tab.innerHTML = "";
    }

    if (TYPE_GRAPH == "scatter") {
      // let t = document.getElementById("container_table"); //inds_table
      // if (t != undefined) t.remove();
      let cont = document.getElementById("container_graph_obj_space"); // container_graph_obj_space , container_object_spce
      cont.innerHTML = "";
      cont.appendChild(GRAPH_OBJ_SPACE);
    }

    // no hay datos
    if (Object.values(ds_fronts).length == 0) {
      Plotly.react(GRAPH_OBJ_SPACE, [get_base_dataset("", TYPE_GRAPH)], get_layout(TYPE_GRAPH), { displaylogo: false, modeBarButtonsToRemove: ["zoom2d", 'lasso2d', "select2d", "toImage", "autoScale2d"] });
    }

    // se resetea el id_actual
    DATA_CONTROL.actual_front = "";

    send_data_front();
  } else {
    alert(`No existe el tipo de gráfica: ${type_g}`)
    return false;
  }
  return true;
}



// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++  narmalización ++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

function change_norm(type_n) {
  if (type_n == "do_norm" || type_n == "deactiv_norm") {
    if (type_n == "do_norm") NORMALIZATION = true;
    if (type_n == "deactiv_norm") NORMALIZATION = false;

    // se resetea el id_actual
    DATA_CONTROL.actual_front = "";

    send_data_front();
  } else {
    alert(`No existe el tipo de normalización: ${type_n}`)
  }
}