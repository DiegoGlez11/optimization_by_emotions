// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------

function createGraph(id_graph, callback_simulation, callback_tooltip,) {
    let config = {
        type: "scatter",
        data: {
            datasets: [
                {
                    type: "line",
                    label: "Roussell",
                    data: [{ x: 0, y: 0.5 }, { x: 1, y: 0.5 }, { x: 0.5, y: 0.5 }, { x: 0.5, y: 0 }, { x: 0.5, y: 1 }],
                    pointBackgroundColor: [],
                    borderColor: "rgba(44, 179, 55, 0.43)",
                    pointRadius: 0,
                },
            ],
        },
        options: {
            onClick: callback_simulation,
            legend: {
                display: false
            },
            scales: {
                x: {
                    min: 0,
                    max: 1,
                },
                y: {
                    min: 0,
                    max: 1,
                }
            },
            animation: false,
            responsive: true,
            maintainAspectRatio: true,
            plugins: {
                legend: {
                    display: true,
                },
                zoom: {
                    pan: {
                        enabled: true,
                        mode: "xy",
                    },
                    zoom: {
                        wheel: {
                            enabled: true,
                            speed: 0.1,
                        },
                        pinch: {
                            enabled: true,
                        },
                        drag: {
                            enabled: true,
                            modifierKey: "ctrl",
                        },
                        limits: {
                            y: { min: -1, max: 2 }
                        },
                        mode: 'xy',
                    }
                },
                tooltip: {
                    callbacks: {
                        footer: callback_tooltip,
                    },
                },
            },
        },
    };

    // config.data.datasets.forEach(function (dataset) {
    //   let col = new Array(dataset.data.length);
    //   for (let i = 0; i < col.length; i++) {
    //     col[i] = "rgba(9, 142, 219, 1)";
    //   }
    //   dataset.borderColor = "rgba(0, 87, 138,1)";
    //   dataset.backgroundColor = "rgba(9, 142, 219, 1)";
    //   dataset.pointBorderColor = "rgba(0, 87, 138,1)";
    //   dataset.pointBackgroundColor = col;
    //   dataset.pointRadius = 6;
    //   dataset.pointBorderWidth = 2;
    //   dataset.pointHoverRadius = 8;
    // });

    let ctx = document.getElementById(id_graph);
    let graph = new Chart(ctx.getContext("2d"), config);

    return graph;
}

let graph_model = createGraph("russell_evaluations");

// muestra todos los dataset
document.getElementById("btn_show_all").addEventListener("click", () => {
    // se desactivan los datasets
    for (let num_video = 1; num_video < graph_model.data.datasets.length; num_video++) {
        graph_model.setDatasetVisibility(num_video, true);
    }
    graph_model.update();
});

// oculta todos los dataset
document.getElementById("btn_hide_all").addEventListener("click", () => {
    // se desactivan los datasets
    for (let num_video = 1; num_video < graph_model.data.datasets.length; num_video++) {
        graph_model.setDatasetVisibility(num_video, false);
    }
    graph_model.update();
});

// elimina todas las predicicones cargadas
document.getElementById("btn_erase_all").onclick = () => {
    graph_model.data.datasets.splice(1);
    graph_model.update();
    console.log(graph_model.data.datasets);
};


function ros_srv_plot_pred() {
    let srv = rosnodejs.nh.advertiseService("show_predictions", "experiment_control_interfaces/show_predictions", (req) => {
        let param = new neurocontroller_database.get_emotion_evaluations.Request();
        param.user_name = req.user_name;
        param.id_pareto_front = req.id_pareto_front;
        param.num_ind = req.num_ind;
        param.storage_pos = req.storage_pos;
        // console.log("show_predictions", param);
        plot_predictions(param);

        return true;
    });
}
ros_srv_plot_pred();




// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------
// -----------------------------------------------------------------

function add_options(select, options) {
    select.innerHTML = "";
    for (let i = 0; i < options.length; i++) {
        let opt = document.createElement("option");

        let str_opt = options[i];
        opt.setAttribute("value", str_opt);
        opt.innerHTML = str_opt;

        // se agrega al select
        select.appendChild(opt)
    }
}


function get_dir_by_type(user_name, type) {
    let dir = "";
    if (type == "trained")
        dir = `${root_dir}/${user_name}/emotion_model/evaluations_path.txt`;

    if (type == "bci")
        dir = `${root_dir}/${user_name}/history/prediction_data.txt`;

    return dir;
}

function get_users(type) {
    return new Promise((resolve, reject) => {
        get_subdirs(root_dir).then((users) => {
            let user_list = [];
            for (let i = 0; i < users.length; i++) {
                let user_name = users[i];

                let dir = get_dir_by_type(user_name, type);
                // se verifica si existe la evaluación
                if (fs.existsSync(dir)) {
                    user_list.push(user_name);
                }

            }
            resolve(user_list);

        });
    });
}


document.getElementById("rad_bci").onclick = () => {
    create_selects("bci");
}
document.getElementById("rad_train").onclick = () => {
    create_selects("trained");
}
create_selects("trained");

var select_users;
function create_selects(type = "trained") {
    let select_user = document.createElement("select");
    let container = document.getElementById("prediction_user");
    container.innerHTML = "";
    container.appendChild(select_user);
    select_users = select_user;

    let num = 0;
    if (type == "trained") num = 4;
    if (type == "bci") num = 3;


    get_users(type).then((user_list) => {
        add_options(select_user, user_list);

        select_user.onchange = () => {
            // selector del usuario
            let user_name = get_selected(select_user);
            change_user(user_name, type, num);
        }

        let user_name = get_selected(select_user);
        change_user(user_name, type, num);
    });
}

var selects_pred;
// controles de carga de las predicciones de los individuos entrenados
function change_user(user_name, type, num_selects) {
    // se carga el archico
    let dir = get_dir_by_type(user_name, type)
    load_object(dir).then((dict_data) => {
        // console.log(dict_data, dir);
        let select_array = [];
        // contenedor
        let container = document.getElementById("predictions_select");
        container.innerHTML = "";

        function update(pos) {
            let dict = dict_data;
            if (pos == 0)
                add_options(select_array[0], Object.keys(dict));

            for (let n_u = 1; n_u < num_selects; n_u++) {
                let sel = get_selected(select_array[n_u - 1]);
                if (pos <= n_u) {
                    add_options(select_array[n_u], Object.keys(dict[sel]));
                }
                dict = dict[sel];
                if (dict == undefined) break;
            }
        }

        for (let n_s = 0; n_s < num_selects; n_s++) {
            let s = document.createElement("select");
            select_array.push(s);
            container.appendChild(s);

            s.onchange = () => {
                update(n_s + 1);
            }

        }

        update(0);
        selects_pred = select_array;
    });
}



// var dataset_pred = [];
let btn_load_pred = document.getElementById("btn_load_pred");
btn_load_pred.onclick = () => {
    let user_name = get_selected(select_users);

    // let data;
    // selecciones  
    let sel = [];
    for (let i = 0; i < selects_pred.length; i++)
        sel[i] = get_selected(selects_pred[i]);

    // let dir = `${root_dir}/${user_name}`;
    if (selects_pred.length == 4) {
        // dir += `/emotion_model/${sel[0]}/${sel[1]}/k_${sel[2]}_fold_${sel[3]}`;

        let param = new neurocontroller_database.get_emotion_evaluations.Request();
        param.user_name = user_name;
        param.model_name = sel[0];
        param.id_emotion_model = sel[1];
        param.num_rep = sel[2];
        param.num_fold = sel[3];

        plot_predictions(param);
    }
    if (selects_pred.length == 3) {
        // dir += `/data_optimized_individuals/${sel[0]}/${sel[1]}`;

        let param = new neurocontroller_database.get_emotion_evaluations.Request();
        param.user_name = user_name;
        param.id_pareto_front = sel[0];
        param.num_ind = parseInt(sel[1].split("_")[1]);
        param.storage_pos = parseInt(sel[2]);

        plot_predictions(param);
    }
}


function plot_predictions(param) {
    // SE CARGAN LAS PREDICCIONES
    load_predictions(param).then((eval_class) => {
        console.log("plot_predictions", eval_class);
        // predicciones de las pruebas
        let str_model = "";
        if (param.id_emotion_model != "") str_model = `${param.id_emotion_model}_rep-${param.num_rep}_fold-${param.num_fold}`;
        else str_model = `${param.id_pareto_front}_ind-${param.num_ind}_exp${param.storage_pos}`;


        if (eval_class["labels"].length > 0) {
            let ds_lab = Object.assign({}, dataset_chartjs);
            ds_lab.data = eval_class["labels"];
            ds_lab.label = `${str_model}`;
            ds_lab["pointBackgroundColor"] = "green";
            graph_model.data.datasets.push(ds_lab);
        }

        if (eval_class["centroide"] != undefined) {
            let ds_cent = Object.assign({}, dataset_chartjs);
            console.log("cent", eval_class["centroide"]);
            ds_cent.data = eval_class["centroide"];
            ds_cent.label = `${str_model}`;
            ds_cent["pointBackgroundColor"] = "blue";
            graph_model.data.datasets.push(ds_cent);
        }

        let ds_f_pred = Object.assign({}, dataset_chartjs);
        ds_f_pred.data = eval_class["fold_predictions"];
        ds_f_pred.label = `${str_model}`;
        ds_f_pred["pointBackgroundColor"] = "rgb(199, 223, 196)";
        graph_model.data.datasets.push(ds_f_pred);

        if (Object.keys(eval_class["test_predictions"]) > 0) {
            let ds_t_pred = Object.assign({}, dataset_chartjs);
            ds_t_pred.data = eval_class["test_predictions"];
            ds_t_pred.label = `${str_model}_test`;
            ds_t_pred["pointBackgroundColor"] = "rgb(223, 209, 196)";

            graph_model.data.datasets.push(ds_t_pred);
        }

        graph_model.update();
    });
}
