use std::ops::Neg;

use num_bigint::BigInt;
use num_traits::cast::ToPrimitive;
use t4_idl_parser::expr::{ConstExpr, Literal, UnaryOpExpr};

#[derive(Debug)]
pub enum ConstValue {
    Integer(BigInt),
    Boolean(bool),
    Char(char),
    String(String),
    FloatingPoint(f64),
}

pub fn eval(expr: &ConstExpr) -> ConstValue {
    match expr {
        ConstExpr::Literal(n) => eval_literal(n),
        ConstExpr::Add(left, right) => {
            arithmetic_op(left, right, &|n1, n2| n1 + n2, &|n1, n2| n1 + n2)
        }
        ConstExpr::Sub(left, right) => {
            arithmetic_op(left, right, &|n1, n2| n1 - n2, &|n1, n2| n1 - n2)
        }
        ConstExpr::Div(left, right) => {
            arithmetic_op(left, right, &|n1, n2| n1 / n2, &|n1, n2| n1 / n2)
        }
        ConstExpr::Mul(left, right) => {
            arithmetic_op(left, right, &|n1, n2| n1 * n2, &|n1, n2| n1 * n2)
        }
        ConstExpr::And(left, right) => boolean_op(left, right, &|n1, n2| n1 && n2),
        ConstExpr::Or(left, right) => boolean_op(left, right, &|n1, n2| n1 || n2),
        ConstExpr::Xor(left, right) => boolean_op(left, right, &|n1, n2| n1 ^ n2),
        ConstExpr::LShift(left, right) => int_op(left, right, &|n1, n2| n1 << n2.to_u64().unwrap()),
        ConstExpr::RShift(left, right) => int_op(left, right, &|n1, n2| n1 >> n2.to_u64().unwrap()),
        ConstExpr::Mod(left, right) => {
            arithmetic_op(left, right, &|n1, n2| n1 % n2, &|n1, n2| n1 % n2)
        }
        ConstExpr::UnaryOp(e) => eval_unary_op(e),
        ConstExpr::ScopedName(n) => todo!(),
    }
}

fn eval_unary_op(expr: &UnaryOpExpr) -> ConstValue {
    match expr {
        UnaryOpExpr::Minus(e) => {
            let n = eval(e);
            match n {
                ConstValue::Integer(n) => ConstValue::Integer(n.neg()),
                ConstValue::FloatingPoint(n) => ConstValue::FloatingPoint(-n),
                _ => panic!("{:?} is not a number", e),
            }
        }
        UnaryOpExpr::Plus(e) => {
            let n = eval(e);
            match n {
                ConstValue::Integer(n) => ConstValue::Integer(n),
                ConstValue::FloatingPoint(n) => ConstValue::FloatingPoint(n),
                _ => panic!("{:?} is not a number", e),
            }
        }
        UnaryOpExpr::Negate(e) => {
            let n = eval(e);
            match n {
                ConstValue::Boolean(n) => ConstValue::Boolean(!n),
                _ => panic!("{:?} is not a boolean", e),
            }
        }
    }
}

fn eval_literal(expr: &Literal) -> ConstValue {
    match expr {
        Literal::Integer(n) => ConstValue::Integer(n.clone()),
        Literal::Boolean(n) => ConstValue::Boolean(*n),
        Literal::Char(n) => ConstValue::Char(*n),
        Literal::String(n) => ConstValue::String(n.clone()),
        Literal::FloatingPoint(n) => ConstValue::FloatingPoint(*n),
        Literal::FixedPoint(_) => unimplemented!(),
    }
}

fn arithmetic_op(
    left: &ConstExpr,
    right: &ConstExpr,
    int_fn: &dyn Fn(BigInt, BigInt) -> BigInt,
    float_fn: &dyn Fn(f64, f64) -> f64,
) -> ConstValue {
    let n1 = eval(left);
    assert!(matches!(
        n1,
        ConstValue::Integer(_) | ConstValue::FloatingPoint(_)
    ));

    let n2 = eval(right);
    assert!(matches!(
        n2,
        ConstValue::Integer(_) | ConstValue::FloatingPoint(_)
    ));

    match (n1, n2) {
        (ConstValue::Integer(n1), ConstValue::Integer(n2)) => ConstValue::Integer(int_fn(n1, n2)),
        (ConstValue::FloatingPoint(n1), ConstValue::FloatingPoint(n2)) => {
            ConstValue::FloatingPoint(float_fn(n1, n2))
        }
        _ => panic!("{:?} or/and {:?} is/are not (a) number(s)", left, right),
    }
}

fn boolean_op(
    left: &ConstExpr,
    right: &ConstExpr,
    func: &dyn Fn(bool, bool) -> bool,
) -> ConstValue {
    let n1 = eval(left);
    assert!(matches!(n1, ConstValue::Boolean(_)));

    let n2 = eval(right);
    assert!(matches!(n2, ConstValue::Boolean(_)));

    match (n1, n2) {
        (ConstValue::Boolean(n1), ConstValue::Boolean(n2)) => ConstValue::Boolean(func(n1, n2)),
        _ => unreachable!(),
    }
}

fn int_op(
    left: &ConstExpr,
    right: &ConstExpr,
    func: &dyn Fn(BigInt, BigInt) -> BigInt,
) -> ConstValue {
    let n1 = eval(left);
    assert!(matches!(n1, ConstValue::Integer(_)));

    let n2 = eval(right);
    assert!(matches!(n2, ConstValue::Integer(_)));

    match (n1, n2) {
        (ConstValue::Integer(n1), ConstValue::Integer(n2)) => ConstValue::Integer(func(n1, n2)),
        _ => unreachable!(),
    }
}
