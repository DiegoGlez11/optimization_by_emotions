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




// //range de mapeo, usado para determinar el tamaño del individuo
// let ini_range_map = 2, end_range_map = 10;
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



// /*
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++ calcula los nuevos límites para chart ++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// */

// function set_new_range(chart) {

//     //se obtienen los valores extremos de las métricas
//     let g_r = get_ranges(chart);

//     let scale = 0.05;
//     let lx = Math.abs(g_r.x.max - g_r.x.min);
//     let ly = Math.abs(g_r.y.max - g_r.y.min);

//     g_r.x.min = g_r.x.min - Math.abs(lx * scale);
//     g_r.x.max = g_r.x.max + Math.abs(lx * scale);
//     g_r.y.min = g_r.y.min - Math.abs(ly * scale);
//     g_r.y.max = g_r.y.max + Math.abs(ly * scale);

//     // console.log("umbral: ", g_r);
//     chart.config.options.scales.x.min = g_r.x.min;
//     chart.config.options.scales.x.max = g_r.x.max;
//     chart.config.options.scales.y.min = g_r.y.min;
//     chart.config.options.scales.y.max = g_r.y.max;
//     chart.update();
// }

// /*
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++ calcula los rangos maximos y minimos de cada eje dada toda la población cargada++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// */
// function get_ranges(chart) {
//     let fronts = chart.data.datasets

//     let minx, maxx, miny, maxy;
//     //minimos basado en el resto de los frentes de pareto cargados
//     for (let i = 1; i < fronts.length; i++) {
//         let ranges = fronts[i]["ranges_individuals"];

//         if (ranges != undefined) {
//             if (i == 1) {
//                 minx = ranges.x.min;
//                 maxx = ranges.x.max
//                 miny = ranges.y.min;
//                 maxy = ranges.y.max;
//             } else {
//                 if (minx > ranges.x.min) minx = ranges.x.min;
//                 if (maxx < ranges.x.max) maxx = ranges.x.max;
//                 if (miny > ranges.y.min) miny = ranges.y.min;
//                 if (maxy < ranges.y.max) maxy = ranges.y.max;
//             }
//         }
//     }
//     // //el rango actual 
//     // if (new_range != undefined) {
//     //     if (minx > new_range.x.min) minx = new_range.x.min;
//     //     if (maxx < new_range.x.max) maxx = new_range.x.max;
//     //     if (miny > new_range.y.min) miny = new_range.y.min;
//     //     if (maxy < new_range.y.max) maxy = new_range.y.max;
//     // }

//     let global_range = { x: { min: minx, max: maxx }, y: { min: miny, max: maxy } };
//     return global_range;
// }
