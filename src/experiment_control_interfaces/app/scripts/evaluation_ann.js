let btn_eval_ann = document.getElementById("btn_eval_ann")


function create_card(eval) {
    let card = document.createElement("div");
    card.setAttribute("class", "card");

    // metadatos
    let cont1 = document.createElement("div");

    let cont_info = document.createElement("div");
    cont_info.innerHTML = create_info(eval["user_name"], eval["id_emotion_model"], eval["num_rep"], eval["size_rep"], eval["num_fold"], eval["size_fold"]);
    cont_info.setAttribute("class", "card_info");
    cont1.appendChild(cont_info);
    cont1.setAttribute("class", "cont1");

    // metricas de regresión

}

let eee = 0;

function load_evals() {
    return new Promise(async (resolve, reject) => {
        let evaluations = {};

        // se obtienen los usuarios
        await get_subdirs(root_dir).then(async (users) => {

            for (let n_user = 0; n_user < users.length; n_user++) {

                let user = users[n_user];
                // dir de la evaluación 
                let dir_eval = `${root_dir}/${user}/emotion_model/evaluations_path.txt`;
                // se verifica si existe la evaluación
                if (fs.existsSync(dir_eval)) {

                    // se cargan las evaluaciones del usuario
                    await load_object(dir_eval).then(async (evals) => {

                        let models = Object.keys(evals);
                        // inicialización de los resultados del usuario
                        if (evaluations[user] == undefined) evaluations[user] = {};

                        evaluations[user] = evals;

                    }).catch(() => {
                        reject();
                        return;
                    });

                    // eee++;
                    // if (eee > 2) {
                    //     // solo para prueba
                    //     resolve(evaluations);
                    //     return;
                    // }
                }
            }
            resolve(evaluations);
        });
    });
}





// se cargan las evaluaciones en la grafica
function update_evals() {

    load_evals().then((evalutions) => {
        if (Object.keys(evalutions).length == 0) return;

        // input de los usuarios
        update_sel_eval("user_name", evalutions).then(() => {
            // update_graph_3d();
        });

        input_user_name.onclick = () => {
            update_sel_eval("model_name", evalutions);
        };

        input_model_name.onclick = () => {
            update_sel_eval("id_model", evalutions);
        };

        input_id_model.onclick = () => {
            update_sel_eval("num_rep", evalutions);
        };

        input_rep.onclick = () => {
            update_sel_eval("num_fold", evalutions);
        };

        // input_fold.addEventListener("click", () => {
        //     update_sel_eval("", evalutions);
        // });

        // boton de carga de las gráficas de las métricas de evaluación
        btn_load_eval.onclick = () => {
            console.log(input_user_name, input_model_name, input_id_model, input_rep, input_fold);
            // console.log(input_user_name, input_model_name, input_id_model);

            // opciones seleccionadas
            let user_name = get_selected(input_user_name);
            let model_name = get_selected(input_model_name);
            let id_emotion_model = get_selected(input_id_model);
            let num_rep = get_selected(input_rep);
            let num_fold = get_selected(input_fold);

            // se cargan las evaluaciones
            let path_model = evalutions[user_name][model_name][id_emotion_model][num_rep][num_fold];
            // se carga los datos de la evaluación del fold
            load_object(`${path_model}/eval_fold.txt`).then((eval_fold) => {
                // metados
                load_object(`${path_model}/metadata.txt`).then((metadata) => {

                    let size_fold = metadata["size_fold"];
                    let size_rep = metadata["size_rep"];
                    let header = create_header(user_name, id_emotion_model, size_rep, num_rep, size_fold, num_fold);

                    // evaluaciones de regresión
                    eval_fold_aux = eval_fold["evaluations"];
                    let list_eval_fold = [eval_fold_aux["loss"].toFixed(4), eval_fold_aux["mae"].toFixed(4), eval_fold_aux["mape"].toFixed(4), eval_fold_aux["mse"].toFixed(4)];
                    let g_fold = create_chart(list_eval_fold, "Evaluaciones de la validación", "bar", [`loss:${list_eval_fold[0]}`, `mae:${list_eval_fold[1]}`, `mape:${list_eval_fold[2]}`, `mse:${list_eval_fold[3]}`]);
                    header.appendChild(g_fold.code_html);

                    // historial de las evaluaciones
                    // entrenamiento
                    let h_tr = create_history(eval_fold["history"], "Entrenamiento", ["loss", "mae", "mape", "mse"]);
                    header.appendChild(h_tr);

                    // historial de validación
                    let h_val = create_history(eval_fold["history"], "Validación", ["val_loss", "val_mae", "val_mape", "val_mse"]);
                    header.appendChild(h_val);

                    // se cargan las evaluaciones del la prueba
                    load_object(`${path_model}/eval_test.txt`).then((eval_test) => {

                        if (Object.keys(eval_test).length > 0) {
                            // // evaluaciones de regresión
                            eval_test = eval_test["evaluations"];
                            let list_eval_test = [eval_test["loss"].toFixed(4), eval_test["mae"].toFixed(4), eval_test["mape"].toFixed(4), eval_test["mse"].toFixed(4)];
                            let g_test = create_chart(list_eval_test, "Evaluaciones de la prueba", "bar", [`loss:${list_eval_test[0]}`, `mae:${list_eval_test[1]}`, `mape:${list_eval_test[2]}`, `mse:${list_eval_test[3]}`]);
                            header.appendChild(g_test.code_html);
                        }

                        // evaluaciones de clase
                        get_evaluations_srv(user_name, model_name, id_emotion_model, num_rep, num_fold).then((eval_class) => {
                            //FOLD
                            let fold = eval_class["eval_fold"];

                            // contenedor
                            let cont_fold = document.createElement("div");
                            cont_fold.setAttribute("class", "container_eval_fold");
                            // metricas
                            create_metrics(fold["metrics"], cont_fold);

                            // roc
                            create_roc(fold["roc"], cont_fold);
                            // // matriz de confusion
                            create_confusion_matrix(fold["confusion_matrix"], cont_fold);

                            // // se agregan las métricas a la tarjeta
                            header.appendChild(cont_fold);
                            // TEST
                            if (Object.keys(eval_class["eval_test"]).length > 0) {
                                let test = eval_class["eval_test"];
                                //contenedor
                                let cont_test = document.createElement("div");
                                cont_test.setAttribute("class", "container_eval_test");
                                // metricas
                                create_metrics(test["metrics"], cont_test);

                                // roc
                                create_roc(test["roc"], cont_test);

                                // matriz de confusion
                                create_confusion_matrix(test["confusion_matrix"], cont_test);

                                header.appendChild(cont_test);
                            }

                            // contenedor de la gráfica
                            let container = document.getElementById("container_cards");
                            container.appendChild(header);

                        }).catch((e) => {
                            console.error("Error al cargar las evaluaciones de clase\n\n", e);
                            return;
                        });
                    }).catch((e) => {
                        console.error(e);
                        return;
                    });
                });
            }).catch((e) => {
                // reject();
                console.error(e);
                return;
            });
        };

        setTimeout(() => {

            input_user_name.value = "s03";
            input_model_name.value = "DeepConvNet";
            input_id_model.value = "emoModelDeepConvNet_1";
            input_rep.value = "1";
            input_fold.value = "1";
            btn_load_eval.click();
        }, 1000);

    });
}



// -------------------------------------------------------------------
// botones de carga de evaluaciones 
// selectores del la evaluacion a cargar

let input_user_name = document.getElementById("sel_eval_user_name");
let input_model_name = document.getElementById("sel_eval_model_name");
let input_id_model = document.getElementById("sel_eval_id_model");
let input_rep = document.getElementById("sel_eval_rep");
let input_fold = document.getElementById("sel_eval_fold");
let btn_load_eval = document.getElementById("btn_load_eval");
let btn_update_3D = document.getElementById("btn_update_graph")

function update_sel_from_array(sel, list, pos, pos_update) {
    return new Promise((resolve, reject) => {
        if (pos > pos_update) {
            resolve();
            return;
        }
        sel.innerHTML = "";

        for (let i = 0; i < list.length; i++) {
            let txt = list[i];

            let opt = document.createElement("option");
            opt.innerHTML = txt;
            opt.value = txt;

            sel.appendChild(opt);
        }
        resolve();
    });
}

function update_sel_eval(name_sel, evalutions) {
    return new Promise((resolve, reject) => {
        let pos = 0;

        if (name_sel == "user_name") pos = 1;
        if (name_sel == "model_name") pos = 2;
        if (name_sel == "id_model") pos = 3;
        if (name_sel == "num_rep") pos = 4;
        if (name_sel == "num_fold") pos = 5;

        // se actualizan los sel
        update_sel_from_array(input_user_name, Object.keys(evalutions), pos, 1).then(() => {
            let user_n = get_selected(input_user_name);

            update_sel_from_array(input_model_name, Object.keys(evalutions[user_n]), pos, 2).then(() => {
                let model_n = get_selected(input_model_name);

                update_sel_from_array(input_id_model, Object.keys(evalutions[user_n][model_n]), pos, 3).then(() => {
                    let id_m = get_selected(input_id_model);

                    update_sel_from_array(input_rep, Object.keys(evalutions[user_n][model_n][id_m]), pos, 4).then(() => {
                        let rep = get_selected(input_rep);

                        update_sel_from_array(input_fold, Object.keys(evalutions[user_n][model_n][id_m][rep]), pos, 6).then(() => {
                            let fold = get_selected(input_fold);
                            resolve();

                        });
                    });
                });
            });
        });
    });
}


// --------------------------------------------------
// creación código html de las evaluaciones

function create_header(user_name, id_emotion_model, size_rep = undefined, num_rep = undefined, size_fold = undefined, num_fold = undefined) {
    let cont1 = document.createElement("div");
    cont1.setAttribute("class", "cont1");

    let id_header = "";
    if (size_rep == undefined || size_fold == undefined || num_fold == undefined || num_rep == undefined) {
        id_header = `${user_name}/${id_emotion_model}/test`;
    } else {
        id_header = `${user_name}/${id_emotion_model}/${num_rep}/${num_fold}`;
    }
    cont1.setAttribute("id", id_header);

    let cont_info = document.createElement("div");
    cont_info.innerHTML = create_info(user_name, id_emotion_model, num_rep, size_rep, num_fold, size_fold);
    cont_info.setAttribute("class", "card_info");
    cont1.appendChild(cont_info);

    let container_content = document.createElement("div");
    container_content.setAttribute("class", "container_graph_evals")
    cont1.appendChild(container_content);
    // return cont1
    return container_content;
}



function create_info(user_name, id_model, num_rep, size_rep, num_fold, size_fold) {
    let str = `
    <div class="metadata_card">
        <div>
            <label>Nombre de usuario:</label>
            <label>${user_name}</label>
        </div>
        <div>
            <label>ID modelo emocional</label>
            <label>${id_model}</label>
        </div>
        <div>
            <label>Repetición:</label>
            <label>${num_rep}/${size_rep}</label>
        </div>
        <div>
            <label>Fold:</label>
            <label>${num_fold}/${size_fold}</label>
        </div>
    </div>`;
    return str;
}


var metric_layout = {
    barmode: 'group',
    autosize: true,
    width: 400,
    height: 300,
    'yaxis': {
        'range': [0.85, 1]
    },
    margin: {
        l: 60,
        r: 10,
        b: 30,
        t: 35,
    }
};

function create_metrics(metrics, global_container) {

    let container = document.createElement("div");
    container.setAttribute("class", "eval_graph")
    global_container.appendChild(container);

    let ds_m = [];
    let m_val = [metrics["recall"], metrics["precision"], metrics["f1_score"]];
    let m_arr = ["recall", "precision", "f1_score"];
    for (let i = 0; i < m_arr.length; i++) {
        let ds = get_base_dataset(m_arr[i], undefined, "bar");
        for (let n_class = 0; n_class < m_val[i].length; n_class++) {
            ds.x.push(`clase ${n_class + 1}`);
            ds.y.push(parseFloat(m_val[i]));
        }
        ds_m.push(ds);
    }

    Plotly.newPlot(container, ds_m, metric_layout);
}

function create_confusion_matrix(all_conf_matrix, c_global) {

    let num_class = Object.keys(all_conf_matrix);

    let names_c;
    if (num_class.length == 4) names_c = ["clase 1", "clase 2", "clase 3", "clase 4"];
    if (num_class.length == 2) names_c = ["Valence", "Arousal"];

    for (let n_class = 0; n_class < num_class.length; n_class++) {
        const conf_matrix = all_conf_matrix[n_class];

        let container = document.createElement("div");
        c_global.appendChild(container);

        // se cambia la precisión a 4 decimales
        let conf_matrix_table = [];
        for (let x = 0; x < conf_matrix["table"].length; x++) {
            conf_matrix_table.push([]);
            for (let y = 0; y < conf_matrix["table"][x].length; y++) {
                conf_matrix_table[x].push(conf_matrix["table"][x][y].toFixed(4));
            }
        }
        let table = create_table(conf_matrix_table, conf_matrix["index"], conf_matrix["columns"], names_c[n_class]);
        table.setAttribute("class", "conf_matrix");

        container.appendChild(table);

        // let values = conf_matrix_table;
        // values.unshift(conf_matrix["columns"]);
        // conf_matrix["index"].unshift("_");

        // let data = [{
        //     type: 'table',
        //     columnorder: [0, 1, 2],
        //     columnwidth: [50, 50, 50],
        //     header: {
        //         values: conf_matrix["index"],
        //         align: "center",
        //         line: { width: 1, color: 'black' },
        //         fill: { color: "grey" },
        //         font: { family: "Arial", size: 12, color: "white" }
        //     },
        //     cells: {
        //         heiht: 80,
        //         values: values,
        //         align: "center",
        //         fill: { color: ['grey', 'white'] },
        //         line: { color: "black", width: 1 },
        //         font: { family: "Arial", size: 11, color: ["black"] }
        //     }
        // }]

        // Plotly.newPlot(container, data, {
        //     title: names_c[n_class],
        // });
    }


}

function create_roc(roc, c_global) {
    let container_roc = document.createElement("div");
    container_roc.setAttribute("class", "roc_evals");
    c_global.appendChild(container_roc);


    let ds_m = [];
    for (let tp in roc["rates"]["tpr"]) {
        let ds = get_base_dataset(`${tp}`, "markers+lines", "scatter");
        // // se cambia la precisión a 4 decimales
        // let tpr = [];
        // let fpr = [];
        // for (let l = 0; l < roc["rates"]["tpr"][tp].length; l++) {
        //     let tp_ = parseFloat(parseFloat(roc["rates"]["tpr"][tp][l]).toFixed(4));
        //     let fp_ = parseFloat(parseFloat(roc["rates"]["fpr"][tp][l]).toFixed(4));

        //     // tpr.push(tp_);
        //     // fpr.push(fp_);
        //     ds.text.push(`AUC: ${parseFloat(roc["auc"][tp]).toFixed(4)}`)
        // }


        // ds.x = fpr;
        // ds.y = tpr;
        ds.y = roc["rates"]["tpr"][tp];
        ds.x = roc["rates"]["fpr"][tp];

        console.log(ds.x, ds.y);
        ds.marker.size = 7;
        ds_m.push(ds);
    }
    Plotly.newPlot(container_roc, ds_m, metric_layout);
    return container_roc;
}

function create_history(history, title, metrics_train) {

    let dat = [];
    let colors = ["rgba(206, 20, 36, 0.7)", "rgba(14, 105, 165, 0.7)", "rgba(36, 105, 22, 0.7)", "rgba(165, 155, 12, 0.7)"];
    // let metrics_train = ["loss", "mae", "mape", "mse"];
    for (let n = 0; n < metrics_train.length; n++) {
        const name_m = metrics_train[n];
        let c = colors[n];

        let d = { label: name_m, data: history[name_m], borderColor: c, backgroundColor: c };
        dat.push(d);

    }

    let grap_t = create_chart(dat, title, "line");

    return grap_t.code_html
}



function create_chart(data_, title, type, labels = undefined, colors = undefined) {
    // se muestran las leyendas
    let is_legend = false;
    if (data_[0] instanceof Object) is_legend = true;

    let n_labels = [];
    if (labels == undefined) {
        let s;
        if (is_legend) s = data_[0].data.length;
        else s = data_.length;

        for (let i = 0; i < s; i++) n_labels.push(i);
    } else {
        n_labels = labels;
    }

    // colores
    let borderColor = "black";
    let backgroundColor;
    if (type == "bar") {
        backgroundColor = [
            'rgba(14, 105, 165, 0.7)',
            'rgba(36, 105, 22, 0.7)',
            'rgba(206, 20, 36, 0.7)',
            'rgba(165, 155, 12, 0.7)',
            'rgba(165, 12, 145, 0.7)',
            'rgba(85, 150, 10, 0.7)',
            'rgba(201, 203, 207, 0.7)',

        ];
    } else {
        backgroundColor = "black";
    }

    let dataset;
    if (is_legend) {
        dataset = {
            labels: n_labels,
            datasets: data_,
        };

    } else {
        // dataset
        dataset = {
            labels: n_labels,
            datasets: [{
                // label: title,
                data: data_,
                borderWidth: 1,
                backgroundColor: backgroundColor,
                borderColor: borderColor,
            }]
        };
    }

    // configuración de la gráfica
    const config = {
        type: type,
        data: dataset,
        options: {
            scales: {
                y: {
                    beginAtZero: true,
                },
            },
            plugins: {
                legend: {
                    display: is_legend,
                },
                title: {
                    display: true,
                    text: title,
                }
            },
        },
    };
    // canvas
    let canva_ = document.createElement("canvas");
    let container = document.createElement("div");
    container.setAttribute("class", "container_graph_");
    container.appendChild(canva_);

    let graph = new Chart(canva_.getContext("2d"), config);
    return { code_html: container, graph: graph };
}


function create_table(matrix_table, index, columns, title) {

    let table = document.createElement("table");

    let tr_h = document.createElement("tr");
    columns = [" "].concat(columns);
    for (let i = 0; i < columns.length; i++) {
        const c = columns[i];
        let t_h = document.createElement("th");
        t_h.innerHTML = columns[i];
        tr_h.appendChild(t_h);
    }
    table.appendChild(tr_h);

    for (let n_row = 0; n_row < matrix_table.length; n_row++) {
        const row = matrix_table[n_row];

        let t_r = document.createElement("tr");
        let th = document.createElement("th");
        th.innerHTML = index[n_row];

        t_r.appendChild(th);
        for (let n_col = 0; n_col < row.length; n_col++) {
            const elem = row[n_col];
            let t_d = document.createElement("td");
            t_d.setAttribute("class", "cell_cm");
            t_d.innerHTML = `${elem}`;
            t_r.appendChild(t_d);
        }
        table.appendChild(t_r);
    }

    let t = document.createElement("label");
    t.innerHTML = title;

    let c = document.createElement("div");
    c.appendChild(t);
    c.appendChild(table);
    return c;
}



// let btn_evals_ann = document.getElementById("btn_update_evals");
// btn_evals_ann.addEventListener("click", () => {
//     update_evals();
// })

// -------------------------------------
update_evals();







