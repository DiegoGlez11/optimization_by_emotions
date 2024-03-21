
const home_dir = require("os").homedir();
const { log } = require("console");
const { type } = require("os");
const path = require('path');
let sep = path.sep;


//----------------------------------------
//--------- crear input con tooltip -----------------
//-----------------------------------------

function add_tooltip_and_blink(elem, tooltip_msg) {
  elem.setAttribute("class", "blink_show");
  let btn_update = add_tooltip(elem, tooltip_msg);
  let c1 = document.createElement("div");
  c1.setAttribute("class", "blink");
  c1.appendChild(btn_update);
  let c2 = document.createElement("div");
  c2.appendChild(c1);

  return c2;
}

function add_tooltip(elem, tooltip_msg) {
  let tooltip = document.createElement("a");
  tooltip.setAttribute("alt", tooltip_msg);
  tooltip.setAttribute("class", "tooltip");

  tooltip.appendChild(elem);
  return tooltip;
}

//------------------
//hace que un elemento hmtl se coloree por un instante
function show_blink(elem, color) {
  // elem.style.visibility = "visible";
  elem.style.backgroundColor = color;
  setTimeout(() => {
    elem.style.backgroundColor = "transparent";
    // elem.style.visibility = "hidden";
  }, 700);
}

//----------------------------------------
//--------- crear input ROS -----------------
//-----------------------------------------
async function create_input_ros(type_input, name, service, msg_type, callback_prepare_msg, callback_ros = undefined, tooltip_txt = "") {
  let elem = document.createElement("div");
  let name_clean = name.replace(/\s/g, '');
  let lab = document.createElement("label");
  lab.setAttribute("for", name_clean);
  lab.innerHTML = name;
  let event_ = undefined;

  if (type_input == "button2") {
    input_change = document.createElement("button");
    event_ = "click";
  }
  if (type_input == "select") {
    let models = ["EEGNet", "DeepConvNet", "ShallowConvNet", "DeepForest"]
    input_change = create_select(models);
    event_ = "change";
  } else {
    input_change = document.createElement("input");
    input_change.setAttribute("type", type_input);
  }
  if (type_input != "button" && type_input != "button2") {
    elem.appendChild(lab);
  }
  //tooltip
  let in_elem = input_change;
  input_change = add_tooltip(input_change, tooltip_txt);

  in_elem.setAttribute("class", "blink_show");
  let blink = document.createElement("div");
  blink.setAttribute("class", "blink");
  blink.appendChild(input_change);
  input_change = blink;

  if (type_input == "button") {
    event_ = "click";
    in_elem.setAttribute("value", name);
  }

  if (type_input == "checkbox") {
    event_ = "click";
  }
  if (type_input == "number" || type_input == "text" || type_input == "file") {
    in_elem.setAttribute("id", name_clean);
    event_ = "keypress";
  }

  //se agrega el evento al input
  input_change.addEventListener(event_, async (e) => {
    if (type_input == "number" || type_input == "text") {
      if (!(e.key == "Enter" || e.keyCode == 13)) return;
    }

    await new Promise((res, rej) => {
      let msg = callback_prepare_msg(in_elem);
      res(msg);
    }).then((msg) => {
      if (msg == undefined) return;

      //conexión con el servicio ROS
      let srv_ = rosnodejs.nh.serviceClient(service, msg_type);
      srv_.call(msg).then((res) => {
        show_blink(blink, color_success);
        if (callback_ros != undefined) callback_ros(res);
      }).catch((e) => {
        show_blink(blink, color_error);
        console.log("Error con el servicio: ", service, " type: ", msg_type, "\n", e);
      });
    });
  });

  elem.appendChild(input_change);

  //funcion para enviar un dato a la red ros
  elem["ros_send_data"] = function (msg) {
    //conexión con el servicio ROS
    let ros_srv = rosnodejs.nh.serviceClient(service, msg_type);
    ros_srv.call(msg).then((res) => {
      if (callback_ros != undefined) callback_ros(res);
      show_blink(blink, color_success);
    }).catch((e) => {
      show_blink(blink, color_error);
      console.log("Error con el servicio: ", service, " type: ", msg_type, "\n", e);
    });
  };
  //devuelve el input element
  elem["get_input"] = () => {
    return in_elem;
  };

  //guarda el tipo de evento
  elem["type_event"] = event_;
  return elem;
}


function create_select(options) {
  let input_change = document.createElement("select");
  for (let i = 0; i < options.length; i++) {
    let opt = document.createElement("option");
    opt.setAttribute("value", options[i].replace(/\s/g, '_'));
    opt.innerHTML = options[i];
    input_change.appendChild(opt);
  }
  return input_change;
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





function get_base_dataset(name, mode_graph = undefined, type = "scatter3d") {
  let mode_g = "markers";
  if (mode_graph != undefined) mode_g = mode_graph;

  let data_emo = {
    x: [],
    y: [],
    z: [],
    type: type,
    mode: mode_g,
    name: name,
    text: [],
    showlegend: true,
    marker: {
      // color: [],
      size: 3,
      opacity: 0.8,
      symbol: [],
      line: {
        width: 0.1,
        // color: [],
      },
    },
  };

  return data_emo;
}

function get_selected(sel) {
  if (sel.options.length == 0) return "";
  return sel.options[sel.options.selectedIndex].value;
}


//-----------------------------------------------------------------
//----------------------------------------------------------------- 
//------ Se agrega el evento a los botones del menú para activarse y desactivarse
let menu_item = document.getElementsByClassName("menu_item");
for (let i = 0; i < menu_item.length; i++) {
  let item = menu_item[i];
  item.addEventListener("click", () => {
    let cont = document.getElementById(item.value);
    cont.classList.toggle("selected_menu");
  });
}




// -----------------------------------------------
// ----------------------------------------------
// pestañas para mostrar las diferentes interfaces
let btn_ts = document.getElementById("btn_graph_eeg");
let btn_eval = document.getElementById("btn_eval_ann");
let btn_pm = document.getElementById("btn_preference_model");
let btn_all_fronts = document.getElementById("btn_graph_all_fronts");
let btn_emo_class = document.getElementById("btn_emo_class");

let prediction_graph = document.getElementById("container_preference_model");
let ts = document.getElementById("container_graph_view");
let evals_ann = document.getElementById("container_eval_ann");
let graph_fronts = document.getElementById("container_all_fronts");
// let distribution_class = document.getElementById("distribution_class");

// se desactivan las ventanas
prediction_graph.style.display = "none";
ts.style.display = "none";
evals_ann.style.display = "";
graph_fronts.style.display = "none";
// distribution_class.style.display = "none";

btn_ts.addEventListener("click", function () {
  evals_ann.style.display = "none";
  prediction_graph.style.display = "none";
  ts.style.display = "";
  graph_fronts.style.display = "none";
  // distribution_class.style.display = "none";
});


btn_pm.addEventListener("click", function () {
  prediction_graph.style.display = "";
  ts.style.display = "none";
  evals_ann.style.display = "none";
  graph_fronts.style.display = "none";
  // distribution_class.style.display = "none";
});

btn_eval.addEventListener("click", function () {
  prediction_graph.style.display = "none";
  ts.style.display = "none";
  graph_fronts.style.display = "none";
  evals_ann.style.display = "";
  // distribution_class.style.display = "none";
});

btn_all_fronts.addEventListener("click", () => {
  prediction_graph.style.display = "none";
  ts.style.display = "none";
  evals_ann.style.display = "none";
  graph_fronts.style.display = "";
  // distribution_class.style.display = "none";
});

// btn_emo_class.addEventListener("click", () => {
//   prediction_graph.style.display = "none";
//   ts.style.display = "none";
//   evals_ann.style.display = "none";
//   graph_fronts.style.display = "none";
//   // distribution_class.style.display = "";
// });




















function is_conn_refused(e) {
  // let k = Object.keys(e);
  // console.log(k);
  // for (let l = 0; l < k.length; l++) {
  //   console.log(e[k[l]]);
  // }
  return e.code.search("CONNREFUSED") >= 0 || e.code.search("EROSAPIERROR") >= 0;
}




function create_input_param(obj_pram) {//type_in, id_param, text, ros_name, ros_type, callback_prepare_param,) {
  let type_in = obj_pram.type;
  let id_param = obj_pram.id;

  if (type_in == undefined || id_param == undefined) {
    console.error(`Error ID param ${id_param}: Falta ingresar el tipo y ID del input`);
    return;
  }

  // return new Promise((resolve, reject) => {
  let event_in = "";
  // evento del input
  if (type_in == "number" || type_in == "text" || type_in == "file") {
    event_in = "keypress";
  }
  if (type_in == "checkbox" || type_in == "radio") {
    event_in = "click";
  }


  // error del tipo de input
  if (event_in == "") {
    // reject();
    console.error(`Error ID param ${id_param}: error al crear el input, no existe el tipo: ${type_in}`);
    return;
  }

  // contenedor del parámetro
  let container = document.createElement("div");

  // label 
  let label = document.createElement("label");
  label.setAttribute("for", id_param);
  label.innerHTML = obj_pram.text;

  // input para agregar el parámetro
  let input = document.createElement("input");
  input.setAttribute("type", type_in);
  if (type_in == "radio") {
    if (obj_pram.name == undefined || obj_pram.value == undefined) {
      console.error(`No hay nombre/valor para enlazar el input radio ${id_param}`);
      return;
    }
    input.setAttribute("name", obj_pram.name);
    input.setAttribute("value", obj_pram.value);
  }

  // id del parámetro
  input.setAttribute("id", id_param);
  input.setAttribute("class", "ros_param");

  // creación del param
  container.appendChild(label);
  if (obj_pram["non_br"] == undefined) container.appendChild(document.createElement("br"));
  container.appendChild(input);

  // carga el parámetro
  container.load_param = function () {
    load_object(`${root_dir}/control_params.txt`).then((file_pram) => {

      // param a cargar
      let val;
      if (type_in == "radio") val = file_pram[obj_pram.name]
      else val = file_pram[id_param];

      // si no existe el parámetro
      if (val == undefined) {
        console.error(`Error ID param ${id_param}: no se puede cargar el parámetro`);
        return;
      }

      // formato
      if (type_in == "number") val = parseFloat(val);

      // se asigna el valor al input
      if (type_in == "checkbox") {
        // if (typeof (val) != "boolean") {
        //   console.error(`Error al cargar el valor del checkbox ${id_param}`);
        //   return;
        // }
        if (val == obj_pram.checked_value)
          input.checked = true;
        else
          input.checked = false;

      } else if (type_in == "radio") {
        if (id_param.search(val) >= 0) input.checked = true;
      } else {
        input.value = val;
      }
    });
  }

  // evento del input 
  let event_input = (e) => {
    // solo cuando se presiona el enter
    if (type_in == "number" || type_in == "text") {
      if (!(e.key == "Enter" || e.keyCode == 13)) {
        return;
      }
    }

    // formato
    let val;
    if (type_in == "number") val = parseFloat(input.value);
    if (type_in == "checkbox") {
      if (input.checked) val = obj_pram.checked_value;
      else val = obj_pram.non_checked_value;
    }
    if (type_in == "radio") val = obj_pram.value;

    // manejo de errores con el valor
    if (val == undefined) {
      console.error(`Error al guardar el valor del parámetro ${id_param}`);
      return;
    }

    // id
    let id_input = id_param;
    input_aux = input;
    if (type_in == "radio") {
      id_input = obj_pram.name;
      input_aux = container
    }

    // ++++++ ROS ++++++
    function send_ros(ros_name, ros_type, callback_prepare_param) {
      // manejo de errores para el uso de ros
      if (ros_name == undefined && ros_type == undefined) {
        show_blink(input_aux, color_success);
        return;
      }

      if (ros_name != undefined && ros_type == undefined) {
        console.error("Falta el TIPO del servicio/topico ROS");
        show_blink(input_aux, color_error);
        return;
      }
      if (ros_name == undefined && ros_type != undefined) {
        console.error(`Error ID param ${id_input}: Falta el NOMBRE del servicio/topico ROS`);
        show_blink(input_aux, color_error);
        return;
      }
      if (callback_prepare_param == undefined) {
        console.error(`Error ID param ${id_input}: Falta el callback para crear la entrada del servicio/topico ROS`);
        show_blink(input_aux, color_error);
        return;
      }

      // se preparan las entradas del input
      let ros_param = callback_prepare_param(input);
      // manejo errores param
      if (ros_param == undefined) {
        console.error(`Error ID param ${id_input}: No se pasaron parámetros ROS en el callback`);
        show_blink(input_aux, color_error);
        return;
      }

      // servicio ROS
      let srv = rosnodejs.nh.serviceClient(ros_name, ros_type);
      srv.call(ros_param).then((res) => {
        show_blink(input_aux, color_success);
        // if (callback_ros != undefined) callback_ros(res);
      }).catch((e) => {

        show_blink(input_aux, color_warning);
        if (is_conn_refused(e)) console.error("Error con el servicio: ", ros_name, " type: ", ros_type, "ID: ", id_param, "\n NO ESTA ACTIVO EL NODO");
        else console.error("Error con el servicio: ", ros_name, " type: ", ros_type, "ID: ", id_param, "\n", e);
      });
    }


    // se guarda el parámetro en el archivo
    change_param(id_input, val).then(() => {
      let ros_name = obj_pram.ros_name;
      let ros_type = obj_pram.ros_type;
      let ros_callback = obj_pram.callback_prepare_param;

      // manejo de errores para el uso de ros
      if (ros_name == undefined && ros_type == undefined) {
        show_blink(input_aux, color_success);
        return;
      }

      // si se manda solo por un medio de comunicación ROS
      if (Array.isArray(ros_name) && Array.isArray(ros_type) && Array.isArray(ros_callback)) {
        let s1 = ros_name.length;
        let s2 = ros_type.length;
        let s3 = ros_callback.length;
        if (s1 == s2 && s2 == s3) {
          for (let i = 0; i < ros_name.length; i++) {
            send_ros(ros_name[i], ros_type[i], ros_callback[i]);
          }
        } else {
          console.error(`El tamaño de los array con los parámetros para enviar con ROS son de diferente tamaño`, s1, s2, s3);
        }
      } else if (typeof (ros_name) == "string" && typeof (ros_type) == "string" && typeof (ros_callback) == "function") {
        send_ros(ros_name, ros_type, ros_callback);
      } else {
        console.error(`No se reconocen los tipos de parámetros para ROS`);
      }
    });
  }


  // listener
  let input_aux;
  input.addEventListener(event_in, event_input);

  // se regustra la funct
  container["ros_function"] = event_input.bind(container);

  // se regresa el input
  return container;

  // });
}





// abre el archivo de parámetros, agrega o cambia el parámetro con el valor especificado
function change_param(key, val) {
  return new Promise((resolve, reject) => {
    // se cargan los params
    load_object(`${root_dir}/control_params.txt`).then((params) => {
      // se cambia el valor
      params[key] = val;
      // se guarda el valor
      COMM_FUNCT.save_object(params, "control_params.txt", root_dir).then(() => {
        resolve();
      }).catch(() => {
        alert(`No se puede cambiar el valor del parámetro ${key}`);
        reject();
      });
    });
  });
}


function load_param(key) {
  return new Promise((resolve, reject) => {
    // se cargan los params
    load_object(`${root_dir}/control_params.txt`).then((params) => {
      // se cambia el valor
      let val = params[key];
      resolve(val);
    });
  });
}


var exec = require('child_process').exec;

function execute_command(command) {
  exec(`${command}`,
    function (error, stdout, stderr) {
      console.log('stdout: ' + stdout);
      console.log('stderr: ' + stderr);
      if (error !== null) {
        console.log('exec error: ' + error);
      }
    });
}

// execute_command("ls");