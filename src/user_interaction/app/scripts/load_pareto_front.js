

// orden de los objetivos
var index_objectives = [0, 1, 2];
//range de mapeo, usado para determinar el tamaño del individuo
let ini_range_map = 15, end_range_map = 60;


let ui_count = 0;

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



function load_pareto_front(id_pareto, storage_pos = undefined, id_pareto_sel = undefined, num_ind_sel = undefined) {
    return new Promise((resolve, reject) => {
        //se bloquea la gráfica de los frentes
        let win_block = document.getElementById("window_block");
        win_block.style.visibility = "visible";

        load_object(root_dir + "/range_objectives.txt").then((range_objs) => {
            if (Object.keys(range_objs) == 0) {
                let txt = "Error no existe el archivo range_objectives.txt el cual contiene los rangos de la población";
                console.error(txt);
                alert(txt);
                reject();
            }

            //cargamos el espacio de los objetivos
            COMM_FUNCT.get_obj_space(id_pareto, false, NORMALIZATION, id_pareto_sel, num_ind_sel).then((data) => {
                // orden de los objetivos
                let pos_obj = get_pos_objectives(index_objectives, TYPE_GRAPH);

                // dataset de la gráfica
                let ds = get_base_dataset(id_pareto, TYPE_GRAPH);

                // valores de los individuos
                ds.x = data["obj_space"][pos_obj.posx];
                ds.y = data["obj_space"][pos_obj.posy];
                ds["obj3"] = data["obj_space"][pos_obj.posz];
                // datos del ind en el proceso evolutivo
                ds["relative_num_ind"] = data["relative_num_ind"];
                ds["relative_id_pareto"] = data["relative_id_pareto"];
                // config general
                ds.marker.symbol = "circle";
                ds.marker.line["color"] = "black";
                ds.showlegend = false;
                ds["textposition"] = 'bottom center';
                ds["storage_pos"] = storage_pos;
                ds["type"] = TYPE_GRAPH;
                ds["id_pareto"] = id_pareto;
                // config por cada ind
                ds["pos_id"] = data["pos_id"];
                ds.marker["color"] = [];
                ds.marker.size = [];
                ds["num_ind_gui"] = [];
                ds["is_selected"] = [];

                // mapeo: tamaño del círculo
                //  r_min y r_max son el límite inferior y superior del tamaño de un círculo.
                //  se utiliza el tercer objetivo como el tamaño del círculo
                let r_min, r_max;
                if (NORMALIZATION) { r_min = 0; r_max = 0.5 }
                else {
                    r_min = range_objs.min[pos_obj.posz];
                    r_max = range_objs.max[pos_obj.posz] * 0.5
                }

                // 1d to 2d
                let val, val3, valx, valy;
                for (let n = 0; n < data["pop_size"]; n++) {
                    // valores del ind
                    valx = ds.x[n];
                    valy = ds.y[n];
                    val3 = ds.obj3[n];

                    // se acortan los dígitos del valor para evitar mostrar números con muchos decimales
                    if (NORMALIZATION) {
                        valx = parseFloat(valx.toFixed(8));
                        valy = parseFloat(valy.toFixed(8));
                        val3 = parseFloat(val3.toFixed(8));
                    }

                    // tamaño del círculo
                    val = map_val(val3, r_min, r_max, ini_range_map, end_range_map);
                    ds.marker.size.push(val);

                    // si es un individuo previamente seleccionado
                    ds.is_selected.push(false);

                    // datos del individuo
                    ds.text.push(`${n + 1}`);
                    ds.num_ind_gui.push(n + 1);
                    ds.marker.color.push(color_point);

                    if (TYPE_GRAPH == "parcoords")
                        ds.line.color.push(0.2);
                }


                // rangos de cada eje de la gráfica
                let range = {};
                if (!NORMALIZATION) {
                    range["x"] = [range_objs.min[pos_obj.posx], range_objs.max[pos_obj.posx]];
                    range["y"] = [range_objs.min[pos_obj.posy], range_objs.max[pos_obj.posy]];
                    range["z"] = [range_objs.min[pos_obj.posz], range_objs.max[pos_obj.posz]];
                } else if (NORMALIZATION) {
                    range["x"] = [0, 1];
                    range["y"] = [0, 1];
                    range["z"] = [0, 1];
                }

                // configuración de la gráfica de coordenadas paralelas
                if (TYPE_GRAPH == "parcoords") {
                    let dim = [];
                    dim.push({ label: pos_obj.names[pos_obj.posx], values: ds.x, range: range["x"] });
                    dim.push({ label: pos_obj.names[pos_obj.posy], values: ds.y, range: range["y"] });
                    dim.push({ label: pos_obj.names[pos_obj.posz], values: ds.obj3, range: range["z"] });
                    ds["dimensions"] = dim;
                    ds["rangefont"] = { size: 0.001, color: "transparent" };

                    // tabla
                    let table = document.getElementById("inds_table");
                    table_set_data(table, ds, id_pareto);
                }

                // se verfica un registro anterior
                let prev_pareto = PARETO_DATA[id_pareto];
                // se reinicia el registro de los datos de los frentes de pareto
                PARETO_DATA = {};

                if (prev_pareto != undefined) {
                    // si anteriormente estaba seleccionado un ind
                    for (let i = 0; i < prev_pareto.is_selected.length; i++) {
                        if (prev_pareto.is_selected[i]) {
                            update_graph(id_pareto, prev_pareto.relative_id_pareto[i], prev_pareto.relative_num_ind[i], prev_pareto.storage_pos, false);
                            ds.is_selected[i] = true;
                        }
                    }
                }

                // se guarda el dataset
                PARETO_DATA[id_pareto] = ds;
                Plotly.react(GRAPH_OBJ_SPACE, Object.values(PARETO_DATA), get_layout(TYPE_GRAPH), { displaylogo: false, modeBarButtonsToRemove: ["zoom2d", 'lasso2d', "select2d", "toImage"] });

                // se registra el frente actual
                CONTROL_DATA.actual_front = id_pareto;

                // se indica que no hay individuo seleccionado
                is_select_ind(false);

                // no hay id de almacenamiento
                if (storage_pos == undefined) {
                    win_block.style.visibility = "hidden";
                    //indicador que se cargó el frente
                    show_blink(GRAPH_OBJ_SPACE, color_success);

                    resolve();
                    return;
                }

                // num of individuals by front
                let size_sel = ds.is_selected.length;

                // se carga la tabla de frentes en el módulo de preferencias
                change_model_preference(size_sel, undefined, undefined, id_pareto).then(() => {
                    win_block.style.visibility = "hidden";
                    //indicador que se cargó el frente
                    show_blink(GRAPH_OBJ_SPACE, color_success);

                    if (EMOTIONS_CAPTURE_MODE == "sam" || EMOTIONS_CAPTURE_MODE == "bci") {
                        if (SELECTION_MODE == "sel_automatic") {
                            send_data_front();
                        }
                    }

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
    let axis_name = [];
    let obj3;
    let complete = 0;

    for (let d = 0; d < index_obj_sel.length; d++) {
        let val = index_obj_sel[d];
        // eje x
        if (val == 0) {
            if (obj_sel[0] == undefined) {
                obj_sel[0] = d;
                name_obj[d] = `Distancia${str_space}recorrida`;
                axis_name[d] = "x";
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
                axis_name[d] = "y";
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
                axis_name[d] = "z";
            } else {
                console.error("Error con el objetivo 2");
                complete--;
            }
        }
    }

    if (complete != 3) {
        throw new Error("Error con la selección de objetivos deben ser 0,1,2 donde:\n 0 es el eje X, 1 el eje Y y 2 el tamaño del círculo\n ingreso" + JSON.stringify(index_obj_sel));
        // return;
    }

    //solo deben seleccionarse dos
    if (obj_sel.length != 2) {
        throw new Error("Solo deben seleccionarse dos objetivos no", obj_sel.length);
        // return;
    }
    //debe seleccionarse el tercer objetivo 
    if (obj3 == undefined) {
        throw new Error("Falta seleccionar el tercer objetivo");
        // return;
    }


    return { posx: obj_sel[0], posy: obj_sel[1], posz: obj3, names: name_obj, axis_name: axis_name };
}


