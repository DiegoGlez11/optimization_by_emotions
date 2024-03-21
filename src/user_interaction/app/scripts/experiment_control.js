

// control del experimento con ROS
function control_experiment() {
    rosnodejs.nh.advertiseService("experiment_control", "user_interaction/experiment_control", (req, res) => {
        let ctl = req.control;

        // abre la ventana de efectos para estimular al individuo
        if (req.start_effects_window) {
            open_effects_window();
            return true;
        }

        // ultimo frente
        if (ctl == "last_front") {
            CONTROL_DATA.last_front_all = true;
            return true;
        }

        let id_pareto = req.id_pareto_front;

        // frente válido
        if (id_pareto == "") {
            console.error("Frente de pareto inválido: ",);
            return false;
        }
        // individuos a cargar del frente
        let num_inds;
        if (req.num_inds.length > 0) num_inds = req.num_inds;

        // se inicia el experimento desde un frente en especifico
        if (ctl.search("start_experiment") >= 0) {
            play_video(id_pareto, num_inds, req.play_video_intro, req.extend_search);
        }

        // se inicia el experimento desde un frente en especifico pero con evalucioanes
        // emocioanles generadas aleatoriamente
        if (ctl.search("test_experiment") >= 0) {
            start_experiment(id_pareto, num_inds, false, req.extend_search);
        }

        // selección de un individuo
        if (ctl.search("select_ind") >= 0) {
            if (num_inds == undefined) {
                console.error("No hay individuo para seleccionar en el frente", id_pareto);
                return;
            }
            select_ind(id_pareto, id_pareto_ind, req.num_inds[0])
        }



        return true;
    });
}
control_experiment();



function start_experiment(id_pareto, sel_inds = undefined, test_interface = false, extend_search = undefined) {
    return new Promise((resolve, reject) => {
        // si es prueba de interfaz
        if (test_interface) {
            INTERFACE_TEST = true;
            // boton de la prueba
            let btn_test = document.createElement("button");
            btn_test.innerHTML = "Salir de la prueba de la interfaz";
            btn_test.setAttribute("id", "btn_exit_test");

            // evento
            btn_test.onclick = () => {
                show_one_window("msg_all_window");
                // texto
                let txt = document.querySelector("#msg_all_window h1")
                txt.innerHTML = "Iniciando el experimento";

                // se elimina el botón
                btn_test.remove();

                setTimeout(() => {
                    // texto anterior
                    txt.innerHTML = "";
                    INTERFACE_TEST = false;
                    start_experiment(id_pareto, sel_inds, false, extend_search);
                }, 4000);

            };

            document.body.appendChild(btn_test);
        }

        change_model_preference(undefined, undefined, undefined, undefined, undefined, INTERFACE_TEST).then(() => {
            show_one_window("graph");
            // reseteo de la interfaz
            reset_interface().then(() => {
                // nuevo ID del experimento
                new_experiment(test_interface).then((storage_pos) => {
                    // se resetea la pos en la tabla de los frentes
                    change_model_preference(undefined, undefined, undefined, id_pareto, extend_search).then(() => {
                        // console.log(Array.isArray(sel_inds), sel_inds);
                        // sin individuos
                        if (sel_inds == undefined || !Array.isArray(sel_inds)) {
                            get_distributed_inds(id_pareto).then((dist) => {
                                load_pareto_front(id_pareto, storage_pos, dist.relative_id_pareto, dist.relative_num_ind);
                                resolve();
                            }).catch((e) => {
                                reject(e);
                            })
                        } else {
                            load_pareto_front(id_pareto, storage_pos, undefined, sel_inds);
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
        }).catch((e) => {
            reject(e)
        })
    });
}


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



function send_data_front(id_pareto = undefined, id_pareto_ind = undefined, num_ind = undefined, storage_pos = undefined) {
    let err = 0;
    if (id_pareto == undefined) err++;
    if (num_ind == undefined) err++;
    if (storage_pos == undefined) err++;
    if (id_pareto_ind == undefined) err++;

    let txt_err = `No se puede ejecutar el individuo: {id_pareto:${id_pareto}, id_pareto_ind:${id_pareto_ind},num_ind:${num_ind}, storage_pos:${storage_pos}}`;
    // console.log(id_pareto, id_pareto_ind, num_ind, storage_pos);
    if (err != 0 && err != 4) throw Error(txt_err);

    // posiciones de los inds en su dataset
    let position, ds;
    let is_last = false;
    let pos_id;

    // get pos of specific ind
    if (err == 0) {
        // get dataset of pareto front
        ds = PARETO_DATA[id_pareto];
        if (ds == undefined) throw Error(txt_err);

        position = ds["pos_id"][id_pareto_ind][`ind_${num_ind}`];

        if (ds.is_selected[position])
            throw (`Ya se seleccionó el individuo. id_pareto:${id_pareto} relative_id_pareto:${id_pareto_ind} num_ind:${num_ind}`);
    }

    // calculate next ind
    if (err == 4) {
        // get dataset of pareto front
        ds = PARETO_DATA[CONTROL_DATA.actual_front];
        if (ds == undefined) throw Error(txt_err);


        for (let i = 0; i < ds.is_selected.length; i++)
            if (!ds.is_selected[i]) position = i;

        if (position == undefined)
            throw (`No hay individuo para mostrar. is_selected:${s.is_selected}`);
    }

    // check if the last ind
    let count_sel = 0;
    for (let i = 0; i < ds.is_selected.length; i++) {
        if (ds.is_selected[i])
            count_sel++;
    }
    if (count_sel == ds.is_selected.length - 1) is_last = true;

    let ind_sel = { dataset: ds, position: position, is_last: is_last };
    let event = new CustomEvent("select_individual", { "detail": ind_sel });
    GRAPH_OBJ_SPACE.dispatchEvent(event);
}


function is_select_ind(is_sel) {
    CONTROL_DATA.select_ind = is_sel;

    let btn_next = document.getElementById("next_individual");
    if (btn_next == undefined) return;

    if (is_sel) {
        btn_next.style.backgroundColor = "red";
    } else {
        btn_next.style.backgroundColor = "green";
    }
}

function f_error_sel(e) {
    console.error(e);
    // desbloqueo
    // CONTROL_DATA.select_ind = false;
    is_select_ind(false);
}

// captura el evento cuando se termina de seleccionar un individuo
GRAPH_OBJ_SPACE.addEventListener("select_individual", (e) => {
    let random = false;

    // datos del frente
    let pareto_front = e.detail;

    // dataset del frente
    let ds = e.detail.dataset;
    // posición del individuo
    let pos = e.detail.position;

    // id del frente
    let id_pareto = ds.id_pareto;
    // id del individuo
    let id_pareto_ind = ds.relative_id_pareto[pos];
    // num del individuo
    let num_ind = ds.relative_num_ind[pos];
    // id del experimento
    let storage_pos = ds.storage_pos;

    let txt_ind = `{id_pareto:${id_pareto}, id_pareto_ind: ${id_pareto_ind},num_ind:${num_ind}, storage_pos:${storage_pos}}`;

    // si ya no hay frentes por mostrar
    if (pareto_front == undefined) {
        console.error(`Dataset no cargado: ${txt_ind}`);
        return;
    }

    // datos completos
    if (id_pareto == undefined || num_ind == undefined || storage_pos == undefined) {
        console.error(`Faltan datos para seleccionar el individuo: ${txt_ind}`);
    }

    if (CONTROL_DATA.select_ind) {
        console.log(`Individuo ejecutándose, no se puede cargar el individuo: ${txt_ind}`);
        return;
    }

    is_select_ind(true);

    // individuo no seleccionado
    if (!ds.is_selected[pos]) {
        select_ind(id_pareto, id_pareto_ind, num_ind, storage_pos, random).then(() => {
            // se indica que ya fue selecciondo
            ds.is_selected[pos] = true;

            // si no hay más individuos del frente actual
            if (pareto_front.is_last) {

                // se manda el punto de referencias
                if (EMOTIONS_CAPTURE_MODE == "traditional") {
                    let f_err = function (err) {
                        let txt = `Error al enviar el individuo preferido`;
                        if (is_conn_refused(err)) console.error(txt);
                        else console.error(`${txt}: \n\n${err}`);
                    }

                    get_actual_user().then((user_name) => {
                        let ind_values = [ds.x[pos], ds.y[pos], ds.obj3[pos]];
                        search_new_front(user_name, ind_values, id_pareto, num_ind, storage_pos).catch((e) => {
                            f_err(e);
                        });
                    }).catch((e) => {
                        f_err(e);
                    });
                }


                // se muestra el slider de la fatiga
                show_fatigue_slider(id_pareto, storage_pos, random).then(() => {
                    // si no hay mas frentes por mostrar
                    if (CONTROL_DATA.last_front_all == true) {
                        show_one_window("lim_neuro");
                        return;
                    }

                    // se muestra la ventana del cálculo de frentes
                    let new_fronts = document.getElementById("window_new_fronts");
                    new_fronts.style.visibility = "visible";

                    // desbloqueo
                    // CONTROL_DATA.select_ind = false;
                    is_select_ind(false);

                    let slider_fatigue = document.getElementById("right_metric");
                    //  si existe un nuevo frente a cargar
                    let str_obj = slider_fatigue.getAttribute("new_front");
                    if (str_obj != undefined) {
                        // se extraen los datos
                        let new_front = JSON.parse(str_obj);
                        //reseteo del atributo
                        slider_fatigue.removeAttribute("new_front");
                        // carga de la población
                        load_pareto_front(new_front.id_pareto, new_front.storage_pos, new_front.id_pareto_inds, new_front.individuals);
                    }

                    // se desactiva la pantalla
                    setTimeout(() => {
                        new_fronts.style.visibility = "hidden";
                        // console.log("event front from fatigue");
                    }, 8000);
                }).catch((e) => {
                    f_error_sel(e);
                });
            } else {
                // desbloqueo
                is_select_ind(false);
                console.log("jjjjjjjjjj");
                if (SELECTION_MODE == "sel_automatic") {
                    // console.log("event front from ind>0");
                    // se dispara evento para seleccionar el siguiente ind
                    send_data_front();
                }
            }
        }).catch((e) => {
            f_error_sel(e);
        });
    } else {
        console.error(`Ya se seleccionó el individuo:`);
    }
});


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++ captura la predicción del módulo de preferencias ++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


// se espera la respuesta del siguiente frente
rosnodejs.nh.subscribe("/next_front_ind", "user_interaction/next_front_ind", (msg) => {
    if (msg.num_individuals.length > 0) {
        let id_pareto = msg.id_pareto_front;
        let storage_pos = msg.storage_position;
        let num_inds = msg.num_individuals;
        let id_pareto_inds = msg.id_pareto_inds;

        let new_front = { id_pareto: id_pareto, id_pareto_inds: id_pareto_inds, individuals: num_inds, storage_pos: storage_pos };
        console.log("new_front", new_front);

        let slider_fatigue = document.getElementById("right_metric");
        slider_fatigue.setAttribute("new_front", JSON.stringify(new_front));
    }
});


// reset del experimento
function reset_experiment() {
    get_actual_user().then((user_name) => {
        let dir_user = `${root_dir}/${user_name}`;
        let dir_histo = `${dir_user}/history`;

        // erase_file(`${dir_histo}/adquisition_data.out`);
        // erase_file(`${dir_histo}/emotional_roadmap.txt`);
        // erase_file(`${dir_histo}/front_direction.txt`);
        // erase_file(`${dir_histo}/selected_individuals.txt`);
        // erase_file(`${dir_histo}/prediction_data.out`);
        // erase_file(`${dir_histo}/histo_rest.txt`);
        // // erase_file(`${dir_histo}/histo_basal_state.txt`);
        // erase_file(`${dir_user}/actual_front.txt`);
        // erase_file(`${dir_user}/point_reference.txt`);
        // erase_file(`${dir_user}/pos_fronts_table.txt`);

        erase_file(`${dir_user}/actual_ind.txt`);
        erase_file(`${dir_user}/customer_satisfied_score.txt`);
        erase_file(`${dir_user}/end_experiment.txt`);

        // se eliminan los historiales
        erase_directory(`${dir_user}/history`);

        // se eliminan los datos
        erase_directory(`${dir_user}/data_optimized_individuals`);
        erase_directory(`${dir_user}/optimized_basal`);
    });
}

function erase_file(path) {
    try {
        fs.unlinkSync(path);
    } catch (err) {
        // console.error("Error al eliminar el archivo\n", err);

    }
}

function erase_directory(dir) {
    fs.rmdir(dir, { recursive: true }, err => {
        if (err) {
            // console.error("Error al eliminar el archivo", err);
            return;
        }
    });
}

function open_effects_window() {
    let aux_w = window.open(`${__dirname}/visual_effects.html`, 'visual_effect', 'width=950,height=1020,contextIsolation=no,nodeIntegration=yes');
}