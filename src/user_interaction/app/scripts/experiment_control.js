
let block_exp = {};


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
            DATA_CONTROL.last_front_all = true;
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

        // // se carga un frente de pareto
        // if (ctl.search("load_pareto_front") >= 0) {
        //     load_pareto_front(id_pareto, num_inds);
        // }

        // se inicia el experimento desde un frente en especifico
        if (ctl.search("start_experiment") >= 0) {
            // if (num_inds == undefined) {
            //     console.error("Inicio de la BCI sin individuos");
            //     return;
            // }

            play_video(id_pareto, num_inds, req.play_video_intro, req.extend_search);
        }

        // se inicia el experimento desde un frente en especifico pero con evalucioanes
        // emocioanles generadas aleatoriamente
        if (ctl.search("test_experiment") >= 0) {
            // if (num_inds == undefined) {
            //     console.error("Inicio de la BCI TEST sin individuos");
            //     return;
            // }
            start_experiment(id_pareto, num_inds, false, req.extend_search);
        }

        // selección de un individuo
        if (ctl.search("select_ind") >= 0) {
            if (num_inds == undefined) {
                console.error("No hay individuo para seleccionar en el frente", id_pareto);
                return;
            }
            select_ind(id_pareto, req.num_inds[0])
        }



        return true;
    });
}
control_experiment();


function get_distribuited_inds(id_pareto) {
    return new Promise((resolve, reject) => {
        let srv = rosnodejs.nh.serviceClient("/get_distributed_individuals", "user_interaction/get_distributed_individuals");
        let param = new user_interaction.get_distributed_individuals.Request();
        param.id_pareto_front = id_pareto;

        srv.call(param).then((res) => {
            resolve(res.num_individuals);
        }).catch((e) => {
            console.error(`Error al obtener los individuos distribuidos sobre el frente ${id_pareto}:\n${e}`);
            reject();
        });
    });
}


function start_experiment(id_pareto, sel_inds = undefined, test_interface = false, extend_search = undefined) {
    return new Promise((resolve, reject) => {
        // si es prueba de interfaz
        if (test_interface) {
            INTERFACE_TEST = true;
            // boton de la prueba
            let btn_test = document.createElement("button");
            btn_test.innerHTML = "Salir de la prueba de la interfaz";
            btn_test.setAttribute("id", "btn_exit_test");

            // evenbto
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
                            get_distribuited_inds(id_pareto).then((individuals) => {
                                console.log(individuals);
                                send_data_front(id_pareto, individuals, storage_pos);
                                resolve();
                            }).catch((e) => {
                                reject(e);
                            })
                        } else {
                            send_data_front(id_pareto, sel_inds, storage_pos);
                            resolve();
                        }



                        // // se ocultan las partes gráficas
                        // document.getElementById("right_metric").style.visibility = "hidden";
                        // document.getElementById("left_metric").style.visibility = "hidden";
                        // document.getElementById("window_block").style.visibility = "hidden";
                        // document.getElementById("window_new_fronts").style.visibility = "hidden";
                        // document.getElementById("window_last_front").style.visibility = "hidden";
                        // DATA_FRONT = [];
                        // DATA_CONTROL.actual_front = "";
                        // DATA_CONTROL.select_ind = false;


                        // // individuos iniciales
                        // get_dir().then((user_dir) => {
                        //     let dir_hist = `${user_dir}/history`;

                        //     // primeros ind seleccionados
                        //     load_object(`${dir_hist}/selected_individuals.txt`).then((histo_sel) => {
                        //         if (histo_sel.length == undefined) histo_sel = [];
                        //         histo_sel.push({ id_pareto_front: id_pareto, num_individuals: sel_inds });
                        //         // registro de los primeros ind seleccionados
                        //         save_object(histo_sel, "selected_individuals.txt", dir_hist).then((e) => {
                        //         }).catch((e) => {
                        //             reject(e);
                        //         });
                        //     }).catch((e) => {
                        //         reject(e);
                        //     });
                        // }).catch((e) => {
                        //     reject(e);
                        // });
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



function send_data_front(id_pareto = undefined, individuals = undefined, storage_pos = undefined) {
    if (id_pareto != undefined && individuals != undefined) {
        DATA_FRONT.push({ id_pareto: id_pareto, individuals: individuals, storage_pos: storage_pos, pos_actual_ind: 0 });
    }
    let event = new CustomEvent("select_individual");
    GRAPH_OBJ_SPACE.dispatchEvent(event);
}

function is_select_ind(is_sel) {
    DATA_CONTROL.select_ind = is_sel;

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
    // DATA_CONTROL.select_ind = false;
    is_select_ind(false);
}



// captura el evento cuando se termina de seleccionar un individuo
GRAPH_OBJ_SPACE.addEventListener("select_individual", (e) => {
    let random = false;

    // datos del frente
    let pareto_front = DATA_FRONT[0];
    // si ya no hay frentes por mostrar
    if (pareto_front == undefined) {
        console.error("No hay ningún frente por mostrar");
        return;
    }

    if (DATA_CONTROL.select_ind) {
        console.log("No se puede iniciar la selección. Individuo ejecutando");
        return;
    }

    // bloqueo
    // DATA_CONTROL.select_ind = true;
    is_select_ind(true);

    // id del frente
    let id_pareto = pareto_front.id_pareto;
    // id del experimento
    let storage_pos = pareto_front.storage_pos;
    // nuevo frente a cargar
    if (DATA_CONTROL.actual_front != id_pareto) {

        let aux_pareto = ds_fronts[id_pareto];
        // console.log(aux_pareto);
        // console.log(id_pareto);

        // nuevo frente a cargar
        load_pareto_front(id_pareto, pareto_front.individuals, storage_pos).then(() => {
            // registro del nuevo frente
            DATA_CONTROL.actual_front = id_pareto;
            // desbloqueo
            // DATA_CONTROL.select_ind = false;
            is_select_ind(false);

            // datos del frente
            let ds = ds_fronts[id_pareto];

            // se colorean los ya seleccionados
            if (aux_pareto != undefined && id_pareto == aux_pareto.id_pareto) {
                for (let i = 0; i < aux_pareto.is_selected.length; i++) {
                    if (aux_pareto.is_selected[i]) {
                        update_graph(id_pareto, aux_pareto.num_ind[i], aux_pareto.storage_pos, false);
                        ds.is_selected[i] = true;
                    }
                }
            }

            if (EMOTIONS_CAPTURE_MODE == "sam" || EMOTIONS_CAPTURE_MODE == "bci") {
                if (SELECTION_MODE == "sel_automatic") {
                    // console.log("event front from load_pareto");
                    send_data_front();
                }
            }
        }).catch((e) => {
            f_error_sel(e);
        });
        return;
    }

    // individuo a seleccionar
    let num_ind = pareto_front.individuals[pareto_front.pos_actual_ind];
    pareto_front.pos_actual_ind++;
    // dataset del frente
    let ds = ds_fronts[id_pareto];
    // posición del individuo
    let pos = pos_ind[id_pareto][`ind_${num_ind}`];

    // individuo no seleccionado
    if (!ds.is_selected[pos]) {
        // se cambia el num de individuos esperados en el módulo de preferencias
        change_model_preference(ds.num_ind.length).then(() => {

            select_ind(id_pareto, num_ind, storage_pos, true, random).then(() => {
                // se indica que ya fue selecciondo
                ds.is_selected[pos] = true;

                // si no hay más individuos del frente actual
                if (pareto_front.individuals.length == pareto_front.pos_actual_ind) {

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
                        // se muestra la ventana del cálculo de frentes
                        let new_fronts = document.getElementById("window_new_fronts");
                        new_fronts.style.visibility = "visible";
                        // se elimina el frente actual
                        DATA_FRONT.splice(0, 1);
                        // desbloqueo
                        // DATA_CONTROL.select_ind = false;
                        is_select_ind(false);

                        send_data_front();

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
                    // DATA_CONTROL.select_ind = false;
                    is_select_ind(false);

                    if (SELECTION_MODE == "sel_automatic") {
                        // console.log("event front from ind>0");
                        // se dispara evento para seleccionar el siguiente ind
                        send_data_front();
                    }
                }
            }).catch((e) => {
                f_error_sel(e);
            });
        }).catch((e) => {
            f_error_sel(e);
        });
    } else {
        console.error(`Ya se seleccionó el individuo: ${id_pareto} ind_${num_ind}`);
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

        // se agrega a la fila
        DATA_FRONT.push({ id_pareto: id_pareto, individuals: num_inds, storage_pos: storage_pos, pos_actual_ind: 0 });

        // console.log("event front from ros");
        // se lanza la señal
        send_data_front()
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