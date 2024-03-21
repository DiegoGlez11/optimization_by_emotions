




















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


// ++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++






// function event_load_front(id_pareto, selected_inds) {
//     let event = new CustomEvent("load_front");
//     GRAPH_OBJ_SPACE.dispatchEvent(event, { detailed: { id_pareto: id_pareto, individuals: selected_inds } });
// }

// GRAPH_OBJ_SPACE.addEventListener("load_front", () => {

// });




// function create_pareto_structure(id_pareto) {
//     let container = document.createElement("div");
//     container.setAttribute("id", id_pareto);
//     container.setAttribute("class", `container_pareto ${id_pareto}`);

//     let title = document.createElement("label");
//     title.innerHTML = id_pareto;
//     let cont_title = document.createElement("div");
//     cont_title.setAttribute("class", "title_ind");
//     cont_title.appendChild(title);
//     container.appendChild(cont_title);

//     let container_ind = document.createElement("div");
//     container_ind.setAttribute("class", "container_ind");
//     container.appendChild(container_ind);

//     return container;
// }



// function create_ind_view(num_ind, id_pareto) {
//     let container = document.createElement("div");
//     container.setAttribute("class", `view_ind`);
//     container.setAttribute("id", `${id_pareto}/ind_${num_ind}`);

//     let title = document.createElement("label");
//     title.innerHTML = `ind_${num_ind}`;
//     title.setAttribute("value", id_pareto)
//     container.appendChild(title);

//     return container;
// }


// let step_to_low = 7;
// //en milisegundos
// let time_to_low = 400;

// function update_indicators_ind(id_pareto, pos_objective, individuals_sel = undefined) {
//     // se cargan los rangos de los objetivos

//     load_object(root_dir + "/range_objectives.txt").then((range_objs) => {
//         // rango global del objetivo 3
//         let max, min;
//         if (range_objs["min"] != undefined || range_objs["max"] != undefined) {
//             console.log("Se cargan los rangos de los objetivos desde un archivo externo");
//             max = range_objs["max"][pos_objective];
//             min = range_objs["min"][pos_objective];
//         } else {
//             console.log("Se calcula el rango de los objetivos con los datos cargados");
//             for (let n_front = 1; n_front < size && n_front >= 1; n_front++) {
//                 let range = globalChartRef.data.datasets[n_front]["range_obj3"];
//                 if (n_front == 1) {
//                     max = range.max;
//                     min = range.min;
//                 } else {
//                     if (max < range.max) max = range.max;
//                     if (min > range.min) min = range.min;
//                 }
//             }
//         }

//         //rangos del objetivo 3 global
//         let range_obj3 = { max: max, min: min };
//         globalChartRef.data["range_obj3_global"] = range_obj3;

//         //tamaño del circulo
//         let size = globalChartRef.data.datasets.length;
//         for (let n_front = 0; n_front < size; n_front++) {
//             let all_ind = globalChartRef.data.datasets[n_front].data.length;
//             for (let n_ind = 0; n_ind < all_ind; n_ind++) {
//                 let v = globalChartRef.data.datasets[n_front].data[n_ind]["obj3"];
//                 let p = map_val(v, range_obj3.min, range_obj3.max, ini_range_map, end_range_map);
//                 // console.log(p);
//                 globalChartRef.data.datasets[n_front].pointRadius[n_ind] = p;
//                 globalChartRef.data.datasets[n_front].origin_size[n_ind] = p;
//                 globalChartRef.data.datasets[n_front].pointHoverRadius[n_ind] = p + (p / 4);
//             }
//         }
//         globalChartRef.update();

//         // se agregan las celdas de los individuos si aún no se cargan
//         if (tree_pos[id_pareto] != undefined) {

//             //creamos el código html del contenedor gráfico
//             let cont_view = create_pareto_structure(id_pareto);
//             let size_p = globalChartRef.data.datasets[tree_pos[id_pareto]].data.length;

//             //cada punto se le agrega la acción de crecer y encogerse
//             for (let i = 0; i < size_p; i++) {
//                 let cont_ind = globalChartRef.data.datasets[tree_pos[id_pareto]].data[i];

//                 let ind = create_ind_view(cont_ind.num_ind, id_pareto);
//                 ind.setAttribute("value", JSON.stringify(cont_ind));
//                 cont_view.childNodes[1].appendChild(ind);

//                 // eventos
//                 let interval;
//                 let count = 0;
//                 let l1, l2, l;
//                 //actualización del tamaño del punto
//                 let to_low = function () {
//                     // tamaño del punto
//                     let o_size = globalChartRef.data.datasets[tree_pos[id_pareto]].origin_size[i];

//                     if (count <= step_to_low) {

//                         // tamaño del circulo
//                         let n_size = (l * (step_to_low - count)) + o_size;
//                         // n_size = map_val(n_size, range_obj3.min, range_obj3.max, 0, size_axis);

//                         globalChartRef.data.datasets[tree_pos[id_pareto]].pointRadius[i] = n_size;
//                         globalChartRef.update();

//                         count++;
//                     } else {
//                         count = 0;
//                         clearInterval(interval);
//                     }
//                 }

//                 let n_split = 50;
//                 let grow_ind = () => {
//                     l1 = (globalChartRef.config.options.scales.x.max - globalChartRef.config.options.scales.x.min);
//                     l2 = (globalChartRef.config.options.scales.y.max - globalChartRef.config.options.scales.y.min);
//                     if (l1 < l2) {
//                         // l = l1;
//                         l = globalChartRef.chartArea.width / n_split;
//                         // l = map_val(globalChartRef.chartArea.width / n_split, 0, globalChartRef.chartArea.width, 0, l1);
//                     }
//                     else {
//                         l = globalChartRef.chartArea.height / n_split;
//                         // l = l2;
//                         // l = map_val(globalChartRef.chartArea.height / n_split, 0, globalChartRef.chartArea.height, 0, l2);
//                     }

//                     let time_step = parseInt(time_to_low / step_to_low);
//                     clearInterval(interval);
//                     interval = setInterval(to_low, time_step);
//                     // show_blink(ind, color_success);
//                 };

//                 ind.addEventListener("mouseover", grow_ind);
//                 ind.addEventListener("click", grow_ind);
//             }

//             document.getElementById("container_individuals").appendChild(cont_view);

//             // se colorean los individuos
//             if (individuals_sel != undefined) {
//                 for (let n_ind = 0; n_ind < individuals_sel.length; n_ind++) {
//                     let n_g = individuals_sel[n_ind];
//                     document.getElementById(`${id_pareto}/ind_${n_g}`).style.background = "red";
//                 }
//             }
//         }

//     });
// }









// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

