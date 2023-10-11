

// index_objectives refleja los objetivos a usar, los valores 0 y 1 se usa para formar un punto (x,y) respectivamente
// y el 2 se usa para usar el objetivo como el tamaño del círculo. Los objetivos reflejan: 
//distancia total(trayectoria), min riesgo (inverso de la distancia mas cercana a un objeto), vel contacto 
var index_objectives = [0, 1, 2];

let ui_count = 0;

//range de mapeo, usado para determinar el tamaño del individuo
let ini_range_map = 15, end_range_map = 60;

let GRAPH_OBJ_SPACE = document.getElementById("graph_object_space");



function get_layout(type_g = "scatter") {
    let pos_obj = get_pos_objectives(index_objectives, TYPE_GRAPH);

    let height;
    let width = document.getElementById("container_object_spce").offsetWidth;
    let autosize = true;
    let margin = { l: 60, r: 20, b: 60, t: 35, };
    if (type_g == "parcoords") {
        margin = { l: 60, b: 20, t: 45, };
        // width = 578;
        // height = 450;

        let c_t = document.getElementById("container_table");
        if (c_t == undefined) autosize = true;
        else {
            width = width - c_t.offsetWidth;
            autosize = false;
        }
    }

    let layout = {
        uirevision: 'true',
        dragmode: "pan",
        autosize: autosize,
        width: width,
        height: height,
        xaxis: {
            autorange: true,
            title: {
                text: pos_obj.names[0],
                font: {
                    family: 'Courier New, monospace',
                    size: 24,
                    color: '#7f7f7f'
                }
            },
            showticklabels: true,
            showgrid: true,
            mirror: 'ticks',
            gridcolor: '#bdbdbd',
            gridwidth: 1,
            zerolinecolor: 'black',
            zerolinewidth: 3,
            showline: true,
        },
        yaxis: {
            autorange: true,
            range: [0, 1],
            title: {
                text: pos_obj.names[1],
                font: {
                    family: 'Courier New, monospace',
                    size: 24,
                    color: '#7f7f7f'
                }
            },
            showticklabels: true,
            showgrid: true,
            gridcolor: '#bdbdbd',
            gridwidth: 1,
            zerolinecolor: 'black',
            zerolinewidth: 3,
            showline: true,
        },
        annotations: [],
        dimensions: [{ range: [0, 1] }, { range: [0, 1] }, { range: [0, 1] }],
        margin: margin,

    };

    layout.uirevision = ui_count;
    ui_count++;
    return layout;
}


function load_pareto_front(id_pareto, individuals_sel = undefined, storage_pos = undefined, only_inds_sel = true) {
    return new Promise((resolve, reject) => {
        //se bloquea la gráfica de los frentes
        let win_block = document.getElementById("window_block");
        win_block.style.visibility = "visible";


        // //si está en la gráfica se elimina y se agrega de nuevo
        // if (ds_fronts[id_pareto] != undefined) {
        //     ds_fronts[id_pareto] = undefined;
        // }
        // por restricción de la bci solo se debe cargar un frente a la vez
        ds_fronts = {};
        pos_ind = {};

        load_object(root_dir + "/range_objectives.txt").then((range_objs) => {
            if (Object.keys(range_objs) == 0) {
                let txt = "Error no existe el archivo range_objectives.txt el cual contiene los rangos de la población";
                console.error(txt);
                alert(txt);
                reject();
            }

            //cargamos el espacio de los objetivos
            get_obj_space(id_pareto, NORMALIZATION, false).then((data) => {
                let obj_space = data.obj_space;
                let num_obj = data.num_obj;
                let pop_size = data.pop_size;
                let num_ind_label = data.num_inds;
                let id_pareto_label = data.id_pareto_fronts;

                // orden de los objetivos
                let pos_obj = get_pos_objectives(index_objectives, TYPE_GRAPH);
                if (pos_obj == undefined) {
                    reject();
                }

                let num_inds;
                if (only_inds_sel && individuals_sel != undefined) {
                    pop_size = individuals_sel.length;
                    num_inds = individuals_sel;
                } else {
                    num_inds = [...Array(pop_size)].map((_, i) => i)
                }
                // pos del ind
                if (pos_ind[id_pareto] == undefined) pos_ind[id_pareto] = {};


                let ds = get_base_dataset(id_pareto, TYPE_GRAPH)
                ds.marker.symbol = "circle";
                ds.marker["color"] = [];
                ds.marker.size = [];
                ds.marker.line["color"] = "black";
                ds.showlegend = false;
                ds["textposition"] = 'bottom center';
                ds["num_ind"] = [];
                ds["num_ind_gui"] = [];
                ds["obj3"] = [];
                ds["is_selected"] = [];
                ds["storage_pos"] = storage_pos;
                ds["type"] = TYPE_GRAPH;
                ds["id_pareto"] = id_pareto;


                // 1d to 2d
                let ini, end, n_ind, ind, val, val3;
                for (let n = 0; n < pop_size; n++) {
                    // num del individuo a cargar
                    n_ind = num_inds[n];

                    // pos de la tupla a extraer
                    ini = n_ind * num_obj;
                    end = ini + num_obj;
                    // se extrae los valores del ind
                    ind = obj_space.slice(ini, end);

                    // valores del eje x y y
                    let valx, valy;
                    valx = ind[pos_obj.posx];
                    valy = ind[pos_obj.posy];
                    // valor del eje z
                    val3 = ind[pos_obj.posz];

                    if (NORMALIZATION) {
                        //     valx = (ind[pos_obj.posx] - range_objs.min[pos_obj.posx]) / (range_objs.max[pos_obj.posx] - range_objs.min[pos_obj.posx]);
                        //     valy = (ind[pos_obj.posy] - range_objs.min[pos_obj.posy]) / (range_objs.max[pos_obj.posy] - range_objs.min[pos_obj.posy]);
                        //     // valor del eje z
                        //     val3 = (ind[pos_obj.posz] - range_objs.min[pos_obj.posz]) / (range_objs.max[pos_obj.posz] - range_objs.min[pos_obj.posz]);

                        valx = parseFloat(valx.toFixed(8));
                        valy = parseFloat(valy.toFixed(8));
                        val3 = parseFloat(val3.toFixed(8));
                    }
                    // } else if (NORMALIZATION == "deactiv_norm") {
                    //     valx = ind[pos_obj.posx];
                    //     valy = ind[pos_obj.posy];
                    //     // valor del eje z
                    //     val3 = ind[pos_obj.posz];
                    // }

                    ds.x.push(valx);
                    ds.y.push(valy);
                    ds.obj3.push(val3);

                    // mapeo: tamaño del círculo
                    // val = map_val(val3, range_objs.min[pos_obj.posz], range_objs.max[pos_obj.posz], ini_range_map, end_range_map);
                    let r_min, r_max;
                    if (NORMALIZATION) { r_min = 0; r_max = 0.5 }
                    else {
                        r_min = range_objs.min[pos_obj.posz];
                        r_max = range_objs.max[pos_obj.posz] * 0.5
                    }

                    val = map_val(val3, r_min, r_max, ini_range_map, end_range_map);

                    ds.marker.size.push(val);
                    // si es seleccionado
                    ds.is_selected.push(false);

                    // datos del ind
                    ds.text.push(`${n}`);
                    ds.num_ind.push(n_ind);
                    ds.num_ind_gui.push(n);
                    ds.marker.color.push(color_point);

                    if (TYPE_GRAPH == "parcoords")
                        ds.line.color.push(0.2);

                    // pos del ind
                    pos_ind[id_pareto][`ind_${n_ind}`] = n;
                }


                // normalization
                let range = {};
                // range["x"] = [Math.min(...ds.x), Math.max(...ds.x)];
                // range["y"] = [Math.min(...ds.y), Math.max(...ds.y)];
                // range["z"] = [Math.min(...ds.obj3), Math.max(...ds.obj3)];


                if (!NORMALIZATION) {
                    range["x"] = [range_objs.min[pos_obj.posx], range_objs.max[pos_obj.posx]];
                    range["y"] = [range_objs.min[pos_obj.posy], range_objs.max[pos_obj.posy]];
                    range["z"] = [range_objs.min[pos_obj.posz], range_objs.max[pos_obj.posz]];
                } else if (NORMALIZATION) {
                    range["x"] = [0, 1];
                    range["y"] = [0, 1];
                    range["z"] = [0, 1];
                }

                if (TYPE_GRAPH == "parcoords") {
                    let dim = [];
                    dim.push({ label: pos_obj.names[pos_obj.posx], values: ds.x, range: range["x"] });
                    dim.push({ label: pos_obj.names[pos_obj.posy], values: ds.y, range: range["y"] });
                    dim.push({ label: pos_obj.names[pos_obj.posz], values: ds.obj3, range: range["z"] });
                    ds["dimensions"] = dim;
                    ds["rangefont"] = { size: 0.001, color: "transparent" };

                    // tabla
                    let table = document.getElementById("inds_table");
                    table_add_data(table, { x: ds.x, y: ds.y, z: ds.obj3 }, ds.num_ind, id_pareto);
                }

                // se guarda el dataset
                ds_fronts[id_pareto] = ds;
                Plotly.react(GRAPH_OBJ_SPACE, Object.values(ds_fronts), get_layout(TYPE_GRAPH), { displaylogo: false, modeBarButtonsToRemove: ["zoom2d", 'lasso2d', "select2d", "toImage"] });

                // // línea usada para que solo se muestre un frente a la vez
                // globalChartRef.data.datasets = [{ data: [], pointBackgroundColor: [0], label: "" }];


                // no hay id de almacenamiento
                if (storage_pos == undefined) {
                    win_block.style.visibility = "hidden";
                    //indicador que se cargó el frente
                    show_blink(GRAPH_OBJ_SPACE, color_success);

                    resolve();
                    return;
                }

                // se carga la tabla de frentes en el módulo de preferencias
                change_model_preference(undefined, undefined, undefined, id_pareto).then(() => {
                    win_block.style.visibility = "hidden";
                    //indicador que se cargó el frente
                    show_blink(GRAPH_OBJ_SPACE, color_success);

                    resolve();
                }).catch((e) => {
                    win_block.style.visibility = "hidden";
                    show_blink(GRAPH_OBJ_SPACE, color_error)
                    reject(e);
                });
            }).catch((e) => {
                win_block.style.visibility = "hidden";
                show_blink(GRAPH_OBJ_SPACE, color_error)
                reject(e);
            });
        });
    });
}



function get_pos_objectives(index_obj_sel, type_g = false) {
    let str_space = " ";
    if (type_g == "parcoords") str_space = "<br>";

    //se buscan los objetivos seleccionados
    // let obj_sel = [undefined, undefined];
    let obj_sel = [];
    let name_obj = [];
    let obj3;
    let complete = 0;

    for (let d = 0; d < index_obj_sel.length; d++) {
        let val = index_obj_sel[d];
        // eje x
        if (val == 0) {
            if (obj_sel[0] == undefined) {
                obj_sel[0] = d;
                name_obj[d] = `Distancia${str_space}recorrida`;
                complete++;
            } else {
                console.error("Error con el objetivo 0");
                complete--;
            }
        }
        // eje y
        if (val == 1) {
            if (obj_sel[1] == undefined) {
                obj_sel[1] = d;
                complete++;
                name_obj[d] = "Riesgo";
            } else {
                console.error("Error con el objetivo 1");
                complete--;
            }
        }
        // tamaño del circulo
        if (val == 2) {
            if (obj3 == undefined) {
                obj3 = d;
                complete++;
                name_obj[d] = `Velocidad${str_space}de contacto`;
            } else {
                console.error("Error con el objetivo 2");
                complete--;
            }
        }
    }

    if (complete != 3) {
        console.error("Error con la selección de objetivos deben ser 0,1,2 donde:\n 0 es el eje X, 1 el eje Y y 2 el tamaño del círculo\n ingreso" + JSON.stringify(index_obj_sel));
        return;
    }

    //solo deben seleccionarse dos
    if (obj_sel.length != 2) {
        console.error("Solo deben seleccionarse dos objetivos no", obj_sel.length);
        return;
    }
    //debe seleccionarse el tercer objetivo 
    if (obj3 == undefined) {
        console.error("Falta seleccionar el tercer objetivo");
        return;
    }


    return { posx: obj_sel[0], posy: obj_sel[1], posz: obj3, names: name_obj };
}







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

