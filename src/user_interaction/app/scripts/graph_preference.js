/*
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
+++++++++++++++++++++++++++++++++++++++++  Grafica  ++++++++++++++++++++++++++++++
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*/
Plotly.react(GRAPH_OBJ_SPACE, [get_base_dataset("", TYPE_GRAPH)], get_layout(TYPE_GRAPH), { displaylogo: false, modeBarButtonsToRemove: ["zoom2d", 'lasso2d', "select2d", "toImage", "autoScale2d"] });

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++ CUANDO SE LE DA CLICK A UN PUNTO EN LA GRÁFICA +++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// solo funciona con scatter y modo tradicional
GRAPH_OBJ_SPACE.on('plotly_click', function (data) {
  if (EMOTIONS_CAPTURE_MODE != "traditional") return;
  if (TYPE_GRAPH != "scatter") return;

  // si hay un individuo ejecutándose
  if (CONTROL_DATA.select_ind) {
    console.error("Ejecutando individuo");
    return;
  }

  // se dió click a un ind
  if (data.points.length > 0) {
    let ind = data.points[0];

    // individuo seleccionado
    let num_ind = ind.data.relative_num_ind[ind.pointIndex];
    // id del frente
    let id_pareto = ind.data.id_pareto;
    // id del experiment
    let storage_pos = ind.data.storage_pos;

    // si es ind seleccionado
    let is_sel = ind.data.is_selected[ind.pointIndex];
    // si ya está seleccionado el ind
    if (is_sel) {
      console.log("Ya se seleccionó el ind", id_pareto, num_ind);
      return;
    }

    // cambio de estado de los puntos a seleccionados
    for (let i = 0; i < ind.data.is_selected.length; i++) {
      if (i != ind.pointIndex) {
        ind.data.is_selected[i] = true;
        update_graph(id_pareto, ind.data.relative_id_pareto[i], ind.data.relative_num_ind[i], false);
      }
    }

    // change color of selected ind
    update_graph(id_pareto, ind.data.relative_id_pareto[ind.pointIndex],
      ind.data.relative_num_ind[ind.pointIndex], true, true);

    // se envía el ind seleccionado
    send_data_front(id_pareto,
      ind.data.relative_id_pareto[ind.pointIndex],
      ind.data.relative_num_ind[ind.pointIndex], storage_pos);
  }
});



// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++ actualiza graficamente un punto en la gráfica ++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

var contador = 0;

function update_graph(id_pareto, id_pareto_ind, num_ind, show_text = true, is_sel = false) {
  load_object(`${root_dir}/range_objectives.txt`).then((range_obj) => {
    // console.log(id_pareto, id_pareto_ind, num_ind, show_text, is_sel);
    // data del ind
    let ds = PARETO_DATA[id_pareto];
    let pos = ds.pos_id[id_pareto_ind][`ind_${num_ind}`];

    let ref, valy, valx, ax, ay;
    if (TYPE_GRAPH == "parcoords") {
      // estilo de la fila
      document.getElementById(`${id_pareto_ind}/ind_${num_ind}`).style.background = "rgb(180, 179, 179)";

      //selección
      ds.dimensions[0].constraintrange = [ds.x[pos] - 0.00000001, ds.x[pos] + 0.00000001];
      // rango de los ejejes
      // ds.dimensions[0].range = [0, 1];
      // ds.dimensions[1].range = [0, 1];
      // ds.dimensions[2].range = [0, 1];

      ds.line.color[pos] = color_sel;//1;//parallel coordinates
      ref = "paper";
      ax = 105;
      ay = -10;

      if (NORMALIZATION)
        valy = ds.x[pos];
      else
        valy = map_val(ds.x[pos], range_obj.min[0], range_obj.max[0], 0, 1);

      valx = 0;
    }

    // color of point
    let color_p;
    if (is_sel)
      color_p = color_sel;
    else color_p = color_block;

    if (TYPE_GRAPH == "scatter") {
      ref = "container";
      ds.marker.color[pos] = color_p;
      valy = ds.y[pos];
      valx = ds.x[pos];
      ax = undefined;
      ay = undefined;
    }

    let layout = get_layout(TYPE_GRAPH);

    let text_sel = "";
    if (show_text) text_sel = `<b>Seleccionado</b>`;

    layout.annotations[0] = {
      x: valx,
      y: valy,
      xref: ref,
      yref: ref,
      text: text_sel,
      // showarrow: true,
      arrowhead: 17,
      ax: ax,
      ay: ay,
    };

    contador++;
    layout.datarevision = contador;
    Plotly.react(GRAPH_OBJ_SPACE, Object.values(PARETO_DATA), layout);

    let win_msg = document.getElementById("txt_ind");
    win_msg.innerHTML = `<h2 style="color: red;">Neurocontrolador: ${PARETO_DATA[id_pareto].num_ind_gui[pos]}</h2>`;
  });
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++  modos de ejecutar la interfaz +++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

function f_mode_bci(id_pareto, num_ind, storage_pos) {
  return new Promise((resolve, reject) => {
    // dirección del usuario
    get_dir().then((dir_user) => {

      //preguntamos por el estado del casco
      get_state_acquisition().then((res) => {
        // hay conexión con una tarjeta
        if (res.connected) {
          //estado basal
          start_basal_state(id_pareto, num_ind, storage_pos).then(() => {

            //se carga el historial de individuos cargados
            let dir_histo = `${dir_user}/history`;
            load_object(`${dir_histo}/adquisition_data.txt`).then((histo) => {

              //inicialización si no existe el historial
              let id_ind = `ind_${num_ind}`;
              if (histo[id_pareto] == undefined) histo[id_pareto] = {};
              if (histo[id_pareto][id_ind] == undefined) histo[id_pareto][id_ind] = {};

              //ADQUISICIÓN DE LAS SEÑALES
              play_eeg(id_pareto, num_ind, storage_pos, is_save_file = true).then(() => {

                //INICIO DE LA SIMULACIÓN
                COMM_FUNCT.simulate_individual(id_pareto, num_ind).then(() => {

                  //SE DETIENE EL FLUJO EEG 
                  stop_eeg().then((res_stop) => {
                    // si no hay storage_pos no se guarda el registro
                    if (storage_pos != undefined) {
                      // registro de las señales EEG
                      let new_reg = {};
                      new_reg["id_eeg_signal"] = res_stop.id_eeg_signal;
                      new_reg["size_signal"] = res_stop.size_experiment;

                      //se agrega al historial
                      histo[id_pareto][id_ind][storage_pos] = new_reg;
                      //actualizamos el historial
                      COMM_FUNCT.save_object(histo, "adquisition_data.txt", dir_histo).then(() => {
                        resolve();
                      }).catch((e) => {
                        reject(e)
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
              }).catch((e) => {
                reject(e);
              });
            });
          }).catch((e) => {
            reject(e);
          });
        } else {
          let txt = "No hay tarjeta conectada, no se puede ejecutar el experimento";
          alert(txt);
          reject(txt);
        }
      }).catch((e) => {
        reject(e)
      });
    }).catch((e) => {
      reject(e)
    });
  });
}

function f_mode_sam(id_pareto, num_ind) {
  return new Promise((resolve, reject) => {

    //INICIO DE LA SIMULACIÓN
    COMM_FUNCT.simulate_individual(id_pareto, num_ind).then(() => {
      resolve();
    }).catch((e) => {
      reject(e);
    });
  });
}

function f_mode_traditional() {
  return new Promise((resolveT) => { resolveT() });
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++ ejecuta un individuo ++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

function select_ind(id_pareto, id_pareto_ind, num_ind, storage_pos = undefined, random = false) {
  return new Promise((resolve, reject) => {
    // update_graph(id_pareto, id_pareto_ind, num_ind);

    // -------------------
    // función de error
    let f_error = function (error) {
      if (error != "") console.error(error);
      document.getElementById("window_block").style.visibility = "hidden";

      show_blink(GRAPH_OBJ_SPACE, color_error);
      reject();
    }

    // ---------------
    // termino del experimento
    function end_select_ind() {
      return new Promise((resolve2, reject2) => {
        // roadmap
        add_to_roadmap(id_pareto, num_ind, storage_pos).then(() => {
          // individuo actual
          update_actual_ind(id_pareto, num_ind, storage_pos).then(() => {

            resolve2();
          }).catch((e) => {
            reject2(e);
          });
        }).catch((e) => {
          reject2(e);
        });
      });
    }


    // ---------------
    // modo a ejecutar
    let mode_function;
    if (EMOTIONS_CAPTURE_MODE == "bci") {
      //se bloquea la gráfica de los frentes
      document.getElementById("window_block").style.visibility = "visible";
      mode_function = f_mode_bci;
    }

    if (EMOTIONS_CAPTURE_MODE == "sam") {
      //se bloquea la gráfica de los frentes
      document.getElementById("window_block").style.visibility = "visible";
      mode_function = f_mode_sam;
    }

    if (EMOTIONS_CAPTURE_MODE == "traditional") {
      mode_function = f_mode_traditional;
    }


    console.error("trabajando con ", id_pareto, num_ind);

    // se ejecuta el flujo dado el modo
    mode_function(id_pareto, num_ind, storage_pos).then(() => {
      // si no hay storage_pos no se guardan los registros
      if (storage_pos == undefined) resolve();

      if (EMOTIONS_CAPTURE_MODE == "traditional") {
        end_select_ind().then(() => {
          resolve();
        }).catch((e) => {
          f_error(`Error: ${e}`);
        });
      } else {
        // slider SAM
        show_emo_slider(id_pareto, id_pareto_ind, num_ind, storage_pos, random).then(() => {
          end_select_ind().then(() => {
            resolve();
          }).catch((e) => {
            f_error(`Error: ${e}`);
          });
        }).catch((e) => {
          f_error(`Error: ${e}`);
        });
      }
    }).catch((e) => {
      f_error(`Error: ${e}`);
    });
  });
}
