use simple_qp::constraint;
use simple_qp::problem_variables::ProblemVariables;
use simple_qp::solver::clarabel_solver::ClarabelSolver;
use simple_qp::solver::Solver;
use simple_qp::expressions::variable::Variable;
use simple_qp::expressions::affine_expression::AffineExpression;

fn eval(a: &Vec<Variable>, t: f32) -> AffineExpression {
    a[0] + a[1]*t + a[2]*t*t+a[3]*t*t*t
}

fn eval_p(a: &Vec<Variable>, t: f32) -> AffineExpression {
    a[1] + 2 * a[2]* t + 3 * a[3]*t*t
}

fn eval_pp(a: &Vec<Variable>, t: f32) -> AffineExpression {
    2*a[2] + 6 * a[3]* t
}

fn cost_acc(a: &Vec<Variable>) -> AffineExpression {
    2 * a[2] + 3 * a[3]
}

fn main() {
    // simple_qp only has scalar or vector decision variables, so limiting
    // ourselves to the x-coordinate here for simplicity
    let points = vec![
        [0.0, 0.0],
        [0.2, 0.8],
        [1.0, 1.0]
    ];

    let mut problem = ProblemVariables::default();
    let a_x = problem.add_vector(4, None, None);
    let b_x = problem.add_vector(4, None, None);

    let objective = cost_acc(&a_x) + cost_acc(&b_x);

    let constraints = vec![
        constraint!(eval(&a_x,0.0) == points[0][0]),
        constraint!(eval(&a_x,1.0) == points[1][0]),
        constraint!(eval(&b_x,0.0) == points[1][0]),
        constraint!(eval(&b_x,1.0) == points[2][0]),
        constraint!(eval_p(&a_x,1.0) == eval_p(&b_x, 0.0)),
        constraint!(eval_pp(&a_x,1.0) == eval_pp(&b_x, 0.0)),
        constraint!(eval_p(&a_x,0.0) == 0.0),
        constraint!(eval_p(&b_x,1.0) == 0.0),
    ];

    let solver = ClarabelSolver::default();
    let res = solver
        .solve(problem, objective, constraints)
        .expect("Solver error");

    let a_x_solution = res.eval_vec(&a_x);
    let b_x_solution = res.eval_vec(&b_x);

    println!("a_x = {:?}, b_x = {:?}", a_x_solution, b_x_solution);
}