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
  if (DATA_CONTROL.select_ind) {
    console.error("Ejecutando individuo");
    return;
  }

  // se dió click a un ind
  if (data.points.length > 0) {
    let ind = data.points[0];

    // individuo seleccionado
    let num_ind = ind.data.num_ind[ind.pointIndex];
    // id del frente
    let id_pareto = ind.data.name;
    // id del experiment
    let storage_pos = ind.data.storage_pos;

    // si es ind seleccionado
    let is_sel = ind.data.is_selected[ind.pointIndex];
    // si ya está seleccionado el ind
    if (is_sel) {
      console.log("Ya se seleccionó el ind", id_pareto, num_ind);
      return;
    }
    console.log(",,,,,,,", typeof num_ind);

    // cambio de estado de los puntos a seleccionados
    for (let i = 0; i < ind.data.is_selected.length; i++) {
      if (i != ind.pointIndex) {
        ind.data.is_selected[i] = true;
        update_graph(id_pareto, ind.data.num_ind[i], false);
      }
    }
    // console.error("click in", id_pareto, "num_ind", num_ind, "num_interfaz", ind.pointIndex, "storage_pos", storage_pos);
    // console.log("event front from click scatter");

    // se eliminan los datos del frente
    DATA_FRONT = [];
    // se envía el ind seleccionado
    send_data_front(id_pareto, [num_ind], storage_pos);

  }
});



// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++ actualiza graficamente un punto en la gráfica ++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

var contador = 0;

function update_graph(id_pareto, num_ind, show_text = true) {
  load_object(`${root_dir}/range_objectives.txt`).then((range_obj) => {

    // pos del ind
    let id_ind = `ind_${num_ind}`;
    let pos = pos_ind[id_pareto][id_ind];
    // data del ind
    let ds = ds_fronts[id_pareto];

    let ref, valy, valx, ax, ay;
    if (TYPE_GRAPH == "parcoords") {
      // estilo de la fila
      document.getElementById(`${id_pareto}/ind_${num_ind}`).style.background = "rgb(180, 179, 179)";

      //selección
      ds.dimensions[0].constraintrange = [ds.x[pos] - 0.00000001, ds.x[pos] + 0.00000001];
      // rango de los ejejes
      // ds.dimensions[0].range = [0, 1];
      // ds.dimensions[1].range = [0, 1];
      // ds.dimensions[2].range = [0, 1];

      ds.line.color[pos] = 1;//parallel coordinates
      ref = "paper";
      ax = 105;
      ay = -10;

      if (NORMALIZATION)
        valy = ds.x[pos];//(valy - range_obj.min[0]) / (range_obj.max[0] - range_obj.min[0]);
      else
        valy = map_val(ds.x[pos], range_obj.min[0], range_obj.max[0], 0, 1);

      // valy = ds.x[pos];

      valx = 0;
    }



    if (TYPE_GRAPH == "scatter") {
      ref = "container";
      ds.marker.color[pos] = "blue";//scatter
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
    Plotly.react(GRAPH_OBJ_SPACE, Object.values(ds_fronts), layout);

    let win_msg = document.getElementById("txt_ind");
    win_msg.innerHTML = `<h2 style="color: red;">Neurocontrolador: ${ds_fronts[id_pareto].num_ind_gui[pos]}</h2>`;
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
                simulate_individual(id_pareto, num_ind).then(() => {

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
                      save_object(histo, "adquisition_data.txt", dir_histo).then(() => {
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
    simulate_individual(id_pareto, num_ind).then(() => {
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

function select_ind(id_pareto, num_ind, storage_pos = undefined, is_publish = true, random = false) {
  return new Promise((resolve, reject) => {

    update_graph(id_pareto, num_ind);

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


    console.error("trabajando con ", id_pareto, num_ind, pos_ind[id_pareto][`ind_${num_ind}`]);

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
        show_emo_slider(id_pareto, num_ind, storage_pos, random).then(() => {
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


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++ ejecuta el siguiente ind +++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// document.getElementById("next_individual").addEventListener("click", () => {
//   if (Object.keys(ds_fronts).length == 0) {
//     alert("No hay individuos cargados");
//   } else {
//     if (SELECTION_MODE == "sel_automatic") return;

//     if (!DATA_CONTROL.select_ind) {
//       // console.log("event front from btn_next_individual");
//       send_data_front();
//     } else {
//       console.error("Ejecutando individuo");
//     }
//   }

// });






















/*
+++++++++++++++++++++++++++++++++++++++
+++++++++ selección de un individuo ++++++++++
+++++++++++++++++++++++++++++++++++++++
*/
// function select_ind(id_pareto, num_ind, is_publish = true, random = false) {
//   return new Promise((resolve, reject) => {

//     update_graph(id_pareto, num_ind);

//     // función de error
//     let f_error = function (error) {
//       if (error != "") console.error(error);
//       document.getElementById("window_block").style.visibility = "hidden";

//       show_blink(GRAPH_OBJ_SPACE, color_error);
//       reject();
//     }


//     //preguntamos por el estado del casco
//     get_state_acquisition().then((res) => {
//       // hay conexión con una tarjeta
//       if (res.connected) {
//         start_basal_state(id_pareto, num_ind).then(() => {
//           console.error("trabajando con ", id_pareto, num_ind);

//           //se bloquea la gráfica de los frentes
//           document.getElementById("window_block").style.visibility = "visible";

//           // dirección del usuario
//           get_dir().then((dir_user) => {
//             //se carga el historial de individuos cargados
//             let dir_histo = `${dir_user}/history`;
//             load_object(`${dir_histo}/adquisition_data.txt`).then((histo) => {

//               //inicialización si no existe el historial
//               let id_ind = `ind_${num_ind}`;
//               if (histo[id_pareto] == undefined) histo[id_pareto] = {};
//               if (histo[id_pareto][id_ind] == undefined) histo[id_pareto][id_ind] = {};

//               // posición de almacenamiento
//               // let storage_pos = histo[id_pareto][id_ind].length;
//               let storage_pos = Object.keys(histo[id_pareto][id_ind]).length;

//               //se carga el número de iteración del experimento
//               load_object(dir_user + "/count_iteration.txt").then((iter) => {
//                 let count = iter.count;

//                 //ADQUISICIÓN DE LAS SEÑALES
//                 play_eeg(id_pareto, num_ind, storage_pos, is_save_file = true).then(() => {

//                   //INICIO DE LA SIMULACIÓN
//                   simulate_individual(id_pareto, num_ind).then(() => {

//                     //SE DETIENE EL FLUJO EEG 
//                     stop_eeg().then((res_stop) => {

//                       // registro de las señales EEG
//                       if (EMOTIONS_CAPTURE_MODE == "bci") {
//                         let new_reg = {};
//                         new_reg["id_eeg_signal"] = res_stop.id_eeg_signal;
//                         new_reg["size_signal"] = res_stop.size_experiment;
//                         new_reg["iteration"] = count;

//                         //se agrega al historial
//                         histo[id_pareto][id_ind][storage_pos] = new_reg;
//                         //actualizamos el historial
//                         save_object(histo, "adquisition_data.txt", dir_histo);
//                       }

//                       //actualizamos la info del individuo actual
//                       load_object(`${dir_user}/actual_ind.txt`).then((actual_ind) => {
//                         actual_ind["id_experiment"] = id_pareto;
//                         actual_ind["num_ind"] = num_ind;
//                         actual_ind["storage_pos"] = storage_pos;

//                         //se guarda el individuo actual
//                         save_object(actual_ind, "actual_ind.txt", dir_user).then(() => {

//                           //se actualiza el historial de los individuos seleccionados
//                           load_object(`${dir_histo}/emotional_roadmap.txt`).then((histo_ind) => {
//                             if (histo_ind.length == undefined) histo_ind = [];


//                             //nuevo registro en el historial
//                             histo_ind.push({ id_pareto_front: id_pareto, num_ind: num_ind, storage_position: storage_pos.toString() });
//                             //se actualiza
//                             save_object(histo_ind, "emotional_roadmap.txt", dir_histo).then(() => {

//                               show_emo_slider(id_pareto, num_ind, storage_pos, random).then(() => {
//                                 //se lanza el evento que indica que ya se terminó la selección del individuo
//                                 let event = new Event("selected_individual");
//                                 GRAPH_OBJ_SPACE.dispatchEvent(event, { detail: {} });

//                                 resolve(storage_pos);
//                               }).catch(() => {
//                                 f_error("Error con los sliders. ()");
//                               });
//                             }).catch(() => {
//                               f_error("Error al detener el flujo EEG. ()");
//                             });
//                           });
//                         }).catch(() => {
//                           f_error("Error al actualizar el individuo actual. ()");
//                         });
//                       });
//                     }).catch(() => 
//                       f_error("Error al detener el flujo EEG. ()");
//                     });
//                   }).catch(() => {
//                     f_error("Error al iniciar la simulación. ()");
//                   });
//                 }).catch(() => {
//                   f_error("Error al obtener las señales EEG. ()");
//                 });;
//               });
//             });
//           }).catch(() => {
//             f_error("Error al obtener la dirección. ()");
//           });
//         });
//       } else {
//         alert("No hay tarjeta conectada, no se puede ejecutar el experimento");
//       }
//     }).catch((e) => {
//       f_error("Error al obtener el estado del casco EEG");
//     });
//   });
// }














// var id_pareto_text;
// var num_ind_text;
// var ind_selected;

// /*
// Crea la grafica del espacio de los objetivos de los neurocontroladores
// */
// function createGraph(id_graph, callback_simulation, callback_tooltip,) {
//   let config = {
//     type: "scatter",
//     data: {
//       datasets: [
//         {
//           label: "",
//           data: [],
//           pointBackgroundColor: [],
//         },
//       ],
//     },
//     options: {
//       onClick: callback_simulation,
//       legend: {
//         display: false
//       },
//       scales: {
//         x: {
//           min: 0,
//           max: 1,
//         },
//         y: {
//           min: 0,
//           max: 1,
//         }
//       },
//       animation: false,
//       responsive: true,
//       maintainAspectRatio: true,
//       plugins: {
//         tooltip: {
//           enabled: false
//         },

//         legend: {
//           display: true
//         },
//         zoom: {
//           pan: {
//             enabled: true,
//             mode: "xy",
//           },
//           zoom: {
//             wheel: {
//               enabled: true,
//               speed: 0.1,
//             },
//             pinch: {
//               enabled: true,
//             },
//             drag: {
//               enabled: true,
//               modifierKey: "ctrl",
//             },
//             limits: {
//               y: { min: -1, max: 2 }
//             },
//             mode: 'xy',
//           }
//         },
//         tooltip: {
//           callbacks: {
//             footer: callback_tooltip,
//           },
//         },
//       },
//     },
//     plugins: [{
//       id: "text_labels",
//       afterDatasetsDraw(chart) {
//         // console.log(chart);
//         // const { ctx_txt } = chart;
//         const ctx_txt = chart.ctx;
//         if (ctx_txt == undefined) { console.log("non context"); return; }

//         ctx_txt.save();
//         if (ind_selected != undefined) {


//           ctx_txt.font = `22px sans - serif`;
//           ctx_txt.fillText(`ind_${ ind_selected.num_ind }`,
//             chart.getDatasetMeta(id_pareto_text).data[num_ind_text].x,
//             chart.getDatasetMeta(id_pareto_text).data[num_ind_text].y);
//           console.log(ind_selected);

//         }
//         // ctx_txt.font = `50px sans - serif`;
//         // ctx_txt.fillText("hahaha", 0, 10);
//         console.log("sucess");
//       },
//     }],
//   };

//   // config.data.datasets.forEach(function (dataset) {
//   //   let col = new Array(dataset.data.length);
//   //   for (let i = 0; i < col.length; i++) {
//   //     col[i] = "rgba(9, 142, 219, 1)";
//   //   }
//   //   dataset.borderColor = "rgba(0, 87, 138,1)";
//   //   dataset.backgroundColor = "rgba(9, 142, 219, 1)";
//   //   dataset.pointBorderColor = "rgba(0, 87, 138,1)";
//   //   dataset.pointBackgroundColor = col;
//   //   dataset.pointRadius = 6;
//   //   dataset.pointBorderWidth = 2;
//   //   dataset.pointHoverRadius = 8;
//   // });

//   let ctx = document.getElementById(id_graph).getContext("2d");
//   let graph = new Chart(ctx, config);

//   graph.data["count_train"] = 0;
//   graph.data["count_predict"] = 0;

//   return graph;
// }

/*
+++++++++++++++++++++++++++++++++++++++
+++++++++ simulacion al dar click a un individuo ++++++++++
+++++++++++++++++++++++++++++++++++++++
*/
// async function simulation_graph(element, dataAtClick) {
//   // se desactiva su uso
//   // return;

//   get_dir().then((dir_user) => {
//     let data = this.data.datasets[0].data;

//     //Se obtiene la posición del usuarios
//     const canvasPosition = Chart.helpers.getRelativePosition(element, globalChartRef);
//     let valX = this.scales["x"].getValueForPixel(canvasPosition.x);
//     let valY = this.scales["y"].getValueForPixel(canvasPosition.y);

//     //Se verifica que se haya hecho click dentro de la grafica
//     let thresholdX = (this.scales["x"].max - this.scales["x"].min) * 0.1;
//     let thresholdY = (this.scales["y"].max - this.scales["y"].min) * 0.1;
//     if (!(valX > this.scales["x"].min - thresholdX &&
//       valX < this.scales["x"].max + thresholdX &&
//       valY > this.scales["y"].min - thresholdY &&
//       valY < this.scales["y"].max + thresholdY)) {
//       return 0;
//     }

//     //-------------- CLICK EN UN ESPACIO VACIO ---------
//     //--------------------------------------------------
//     if (dataAtClick.length == 0) {
//       //punto puesto por el usuario
//       if (globalChartRef.data.datasets[0].data.length == 1) {
//         globalChartRef.data.datasets[0].data = [];
//       }

//       //se agrega el punto de referencia
//       let p = { x: valX, y: valY };
//       globalChartRef.data.datasets[0].data.push(p);
//       globalChartRef.data.datasets[0].pointBackgroundColor[0] = color_point_sel;

//       //se guarda el punto de referencia
//       save_object(p, "point_reference.txt", dir_user);

//       //si se seleccionó un individuo con anterioridad
//       let child_index = globalChartRef.data["child_index"];
//       let pop_index = globalChartRef.data["pop_index"];

//       if (child_index != undefined && pop_index != undefined) {
//         let n_d = globalChartRef.data["pop_index"];
//         let n_c = globalChartRef.data["child_index"];
//         globalChartRef.data.datasets[n_d].pointBackgroundColor[n_c] = globalChartRef.data["pop_color"];

//         //reinicio del ind seleccionado
//         globalChartRef.data["child_index"] = undefined;
//         globalChartRef.data["pop_index"] = undefined;
//         globalChartRef.data["pop_color"] = undefined;
//       }

//       //se actualiza los input
//       document.getElementById("x").value = valX;
//       document.getElementById("y").value = valY;

//       //se actualiza la gráfica
//       globalChartRef.update();

//     } else {
//       //----------- CLICK EN UN INDIVIDUO -------------
//       //-----------------------------------------------

//       //posición del punto seleccionado
//       let n_dataset = dataAtClick[0].datasetIndex;
//       let n_child = dataAtClick[0].index;

//       // punto no válido
//       let p = globalChartRef.data.datasets[n_dataset].pointRadius[n_child];
//       if (p <= 0) return;

//       //el dataset 0 es del punto de referencia, no se puede seleccionar
//       if (n_dataset == 0) return;

//       //se elimina el punto de referencia. Se ha seleccionado un individuo
//       if (globalChartRef.data.datasets[0].data.length == 1) {
//         globalChartRef.data.datasets[0].data = [];
//       }

//       //frente e individuo seleccionado con anterioridad
//       let child_index = globalChartRef.data["child_index"];
//       let pop_index = globalChartRef.data["pop_index"];

//       //se a seleccionado una población diferente
//       let id_actual = `${ n_dataset } - ${ n_child }`
//       let id_old = `${ pop_index } - ${ child_index }`
//       if (id_actual != id_old) {
//         globalChartRef.data["pop_color"] = globalChartRef.data.datasets[n_dataset].pointBackgroundColor[n_child];
//         globalChartRef.data["child_index"] = n_child;
//         globalChartRef.data["pop_index"] = n_dataset;
//       }

//       //Se colorea el punto seleccionado
//       // globalChartRef.data.datasets[n_dataset].pointBackgroundColor[n_child] = color_point_sel;

//       //se pone el valor en los inputs
//       let px = this.data.datasets[n_dataset].data[n_child].x;
//       let py = this.data.datasets[n_dataset].data[n_child].y;
//       document.getElementById("x").value = px;
//       document.getElementById("y").value = py;

//       //se actualiza la gráfica
//       globalChartRef.update();

//       //se inicia el experimento con el individuo
//       let id_pareto = globalChartRef.data.datasets[n_dataset].label;
//       let num_ind = globalChartRef.data.datasets[n_dataset].data[n_child].num_ind;

//       select_ind(id_pareto, num_ind);
//     }
//   }).catch((e) => {
//     console.error("Error al interactuar con la GUI de preferencias\n", e);
//   });
// }




//++++++++ BOTON PARA CENTRAR LA VISTA EN EL ESPACIO DE LOS OBJETIVOS ++++++++
// let btn_pref = document.getElementById("btn_zoom_preference");
// btn_pref.addEventListener("click", () => {
//   set_new_range(globalChartRef);
// });

// ++++++++++++ TOOLTIP +++++++++++++++++
// function tooltip_chart(tooltipItems) {
//   // valores de los individuos
//   let str_rad = "";
//   for (let n_tool = 0; n_tool < tooltipItems.length; n_tool++) {
//     const t_tip = tooltipItems[n_tool];

//     if (t_tip.datasetIndex == 0) {
//       str_rad += "ref_point";
//       continue
//     }
//     if (globalChartRef.data.datasets[t_tip.datasetIndex].pointRadius[t_tip.dataIndex] <= 0) continue;
//     // id del frente de pareto
//     let id_pareto = globalChartRef.data.datasets[t_tip.datasetIndex]["label"];
//     // num ind
//     let num_ind = globalChartRef.data.datasets[t_tip.datasetIndex].data[t_tip.dataIndex].num_ind;

//     //  si no tiene ID entonces no es un individuo 
//     if (id_pareto.search("optimized") < 0) continue;

//     // cambios visuales de la celda del individuo
//     let id_ind = `${ id_pareto } / ind_${ num_ind }`;

//     let cell_ind = document.getElementById(id_ind);
//     if (cell_ind == undefined) {
//       console.error("Error al obtener las celdas de los ind");
//       continue;
//     }
//     // color original
//     // let color_old = globalChartRef.data.datasets[t_tip.datasetIndex].pointBackgroundColor[t_tip.dataIndex];
//     // let color_old = globalChartRef.data.datasets[t_tip.datasetIndex].pointBorderColor[t_tip.dataIndex];
//     let color_old = cell_ind.style.background;

//     // cambio de color
//     cell_ind.style.background = "green";

//     // color original
//     setTimeout(() => {
//       cell_ind.style.background = color_old;
//     }, 300);

//     // texto del tooltip 
//     // let info_ind = `${ id_pareto } / ind_${ num_ind }`;
//     let info_ind = `ind_${ num_ind }`;
//     str_rad += `${ n_tool }: ${ globalChartRef.data.datasets[t_tip.datasetIndex].data[t_tip.dataIndex]["obj3"] } ${ info_ind }\n`;
//   }

//   return str_rad;
// }


// ++++++++++++++++++++++++++++++++++++++++++
// let globalChartRef = createGraph("canva", simulation_graph, tooltip_chart);







