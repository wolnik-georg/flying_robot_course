use cvxrust::prelude::*;
use cvxrust::atoms::index;

fn eval(a: &Expr, t: f64) -> Expr {
    let a0 = index(&a, 0);
    let a1 = index(&a, 1);
    let a2 = index(&a, 2);
    let a3 = index(&a, 3);

    a0 + a1*t + a2*t*t+a3*t*t*t
}

fn eval_p(a: &Expr, t: f64) -> Expr {
    let a1 = index(&a, 1);
    let a2 = index(&a, 2);
    let a3 = index(&a, 3);

    a1 + 2.0 * a2* t + 3.0 * a3*t*t
}

fn eval_pp(a: &Expr, t: f64) -> Expr {
    let a2 = index(&a, 2);
    let a3 = index(&a, 3);

    2.0*a2 + 6.0 * a3* t
}

fn cost_acc(a: &Expr) -> Expr {
    let a2 = index(&a, 2);
    let a3 = index(&a, 3);

    2.0 * a2 + 3.0 * a3
}

fn main() {
    // cvxrust seems to have limited slicing for 2D decision variables
    // We limit ourselves to the x-coordinate here for simplicity
    let points = vec![
        [0.0, 0.0],
        [0.2, 0.8],
        [1.0, 1.0]
    ];
    let a_x = variable(4);
    let b_x = variable(4);

    let solution = Problem::minimize(cost_acc(&a_x) + cost_acc(&b_x))
        .subject_to([
            constraint!((eval(&a_x,0.0)) == (points[0][0])),
            constraint!((eval(&a_x,1.0)) == (points[1][0])),
            constraint!((eval(&b_x,0.0)) == (points[1][0])),
            constraint!((eval(&b_x,1.0)) == (points[2][0])),
            constraint!((eval_p(&a_x,1.0)) == (eval_p(&b_x, 0.0))),
            constraint!((eval_pp(&a_x,1.0)) == (eval_pp(&b_x, 0.0))),
            constraint!((eval_p(&a_x,0.0)) == (0.0)),
            constraint!((eval_p(&b_x,1.0)) == (0.0)),
        ])
        .solve()
        .unwrap();

    println!("Optimal value: {}", solution.value.unwrap());
    println!("a_x = {:?}", &solution[&a_x]);
    println!("b_x = {:?}", &solution[&b_x]);
}
