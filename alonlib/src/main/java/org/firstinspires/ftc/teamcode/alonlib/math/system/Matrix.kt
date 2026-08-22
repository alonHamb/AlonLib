package org.firstinspires.ftc.teamcode.alonlib.math.system

import org.ejml.simple.SimpleMatrix
import org.firstinspires.ftc.teamcode.alonlib.robotPrintError
import kotlin.math.pow

/**
 * A dense, real-valued matrix, backing the ported LQR/Kalman-filter/[LinearSystem] code.
 *
 * Wraps [SimpleMatrix] (EJML) rather than porting WPILib's own `Matrix<Rows, Cols>` -- that type's
 * whole point is compile-time-checked dimensions via a `Nat<N>` phantom-type system (~20 generated
 * `NatN` classes plus a `Num` interface), which isn't worth porting for how little of this codebase
 * does state-space control. Dimensions here are checked at runtime instead, via [robotPrintError].
 */
class Matrix private constructor(internal val storage: SimpleMatrix) {

    constructor(rows: Int, cols: Int) : this(SimpleMatrix(rows, cols))
    constructor(data: Array<DoubleArray>) : this(SimpleMatrix(data))

    val rows get() = storage.getNumRows()
    val cols get() = storage.getNumCols()

    operator fun get(row: Int, col: Int) = storage.get(row, col)
    operator fun set(row: Int, col: Int, value: Double) = storage.set(row, col, value)

    /** Overwrites row [row] in place with [values] (must have exactly [cols] entries). */
    fun setRow(row: Int, vararg values: Double) {
        if (values.size != cols) {
            robotPrintError("setRow got ${values.size} values for a $cols-column matrix")
            return
        }
        for (c in values.indices) storage.set(row, c, values[c])
    }

    /** Overwrites every element in place with [value]. */
    fun fill(value: Double) {
        for (r in 0 until rows) for (c in 0 until cols) storage.set(r, c, value)
    }

    /** Copies [other] into this matrix in place, starting at ([startRow], [startCol]). */
    fun assignBlock(startRow: Int, startCol: Int, other: Matrix) {
        storage.insertIntoThis(startRow, startCol, other.storage)
    }

    /** The [numRows]x[numCols] submatrix starting at ([startRow], [startCol]). */
    fun block(numRows: Int, numCols: Int, startRow: Int, startCol: Int) =
        Matrix(storage.extractMatrix(startRow, startRow + numRows, startCol, startCol + numCols))

    /** The largest absolute value of any element. */
    fun maxAbs(): Double {
        var max = 0.0
        for (r in 0 until rows) for (c in 0 until cols) max = kotlin.math.max(max, kotlin.math.abs(storage.get(r, c)))
        return max
    }

    operator fun plus(other: Matrix): Matrix {
        if (rows != other.rows || cols != other.cols) {
            robotPrintError("Matrix dimension mismatch in plus: ($rows,$cols) + (${other.rows},${other.cols})")
            return this
        }
        return Matrix(storage.plus(other.storage))
    }

    operator fun minus(other: Matrix): Matrix {
        if (rows != other.rows || cols != other.cols) {
            robotPrintError("Matrix dimension mismatch in minus: ($rows,$cols) - (${other.rows},${other.cols})")
            return this
        }
        return Matrix(storage.minus(other.storage))
    }

    operator fun unaryMinus() = Matrix(storage.negative())

    /** Matrix multiplication -- `this * other`, requiring `this.cols == other.rows`. */
    operator fun times(other: Matrix): Matrix {
        if (cols != other.rows) {
            robotPrintError("Matrix dimension mismatch in times: ($rows,$cols) * (${other.rows},${other.cols})")
            return this
        }
        return Matrix(storage.mult(other.storage))
    }

    operator fun times(scalar: Double) = Matrix(storage.scale(scalar))
    operator fun div(scalar: Double) = Matrix(storage.divide(scalar))

    fun elementTimes(other: Matrix) = Matrix(storage.elementMult(other.storage))
    fun elementDiv(other: Matrix) = Matrix(storage.elementDiv(other.storage))

    fun transpose() = Matrix(storage.transpose())

    fun copy() = Matrix(storage.copy())

    /** Overwrites column [col] in place with [column] (an `n`x`1` matrix). */
    fun setColumn(col: Int, column: Matrix) {
        for (r in 0 until rows) storage.set(r, col, column[r, 0])
    }

    /** Column [col] as an `n`x`1` matrix. */
    fun column(col: Int) = block(rows, 1, 0, col)

    /**
     * The lower-triangular Cholesky factor `L` such that `L*Lᵀ = this`. Requires a symmetric
     * positive-definite matrix; prints an error and returns a same-size zero matrix otherwise.
     */
    fun cholesky(): Matrix {
        val n = rows
        val l = Matrix(n, n)
        for (i in 0 until n) {
            for (j in 0..i) {
                var sum = this[i, j]
                for (k in 0 until j) sum -= l[i, k] * l[j, k]
                if (i == j) {
                    if (sum < -1e-9) {
                        robotPrintError("cholesky: matrix isn't positive semidefinite")
                        return zeros(n, n)
                    }
                    l[i, j] = kotlin.math.sqrt(sum.coerceAtLeast(0.0))
                } else {
                    // A ~zero pivot means this whole direction is degenerate (no spread) for a
                    // positive-semidefinite matrix -- treat dependent entries as zero rather than
                    // dividing by ~zero, instead of requiring strict positive-definiteness.
                    l[i, j] = if (l[j, j] > 1e-12) sum / l[j, j] else 0.0
                }
            }
        }
        return l
    }

    /**
     * The matrix inverse. [rows] must equal [cols] and the matrix must be non-singular; prints an
     * error and returns a same-size zero matrix rather than throwing if it isn't.
     */
    fun inverse(): Matrix {
        if (rows != cols) {
            robotPrintError("cannot invert a non-square ($rows,$cols) matrix")
            return zeros(rows, cols)
        }
        return try {
            Matrix(storage.invert())
        } catch (e: Exception) {
            robotPrintError("matrix is singular and cannot be inverted")
            zeros(rows, cols)
        }
    }

    /** Solves `this * x = b` for `x` (more numerically stable than `this.inverse() * b`). */
    fun solve(b: Matrix): Matrix = Matrix(storage.solve(b.storage))

    /**
     * The Moore-Penrose pseudoinverse -- generalizes [inverse] to non-square/singular matrices via
     * least-squares. Used by e.g. [org.firstinspires.ftc.teamcode.alonlib.math.kinematics.MecanumDriveKinematics]
     * to turn its overdetermined 4-wheel forward-kinematics system into a solvable one.
     */
    fun pseudoInverse(): Matrix = Matrix(storage.pseudoInverse())

    fun det() = storage.determinant()
    fun trace() = storage.trace()

    /**
     * The matrix exponential `e^A`, via scaling-and-squaring with a truncated Taylor series.
     * Upstream WPILib computes this through a JNI call into Eigen's solver, which isn't portable
     * to Android -- this is a from-scratch pure-Kotlin/EJML replacement. [rows] must equal [cols].
     */
    fun exp(): Matrix {
        val n = rows
        val norm = normF()
        var squarings = 0
        var scaled = this
        if (norm > 0.5) {
            squarings = kotlin.math.ceil(kotlin.math.ln(norm / 0.5) / kotlin.math.ln(2.0)).toInt().coerceAtLeast(0)
            scaled = this / 2.0.pow(squarings)
        }

        var result = eye(n)
        var term = eye(n)
        for (k in 1..18) {
            term = (term * scaled) / k.toDouble()
            result += term
        }

        repeat(squarings) { result *= result }
        return result
    }

    /** Repeated squaring: `this` raised to a non-negative integer [exponent]. */
    fun pow(exponent: Int): Matrix {
        var result = eye(rows)
        repeat(exponent) { result *= this }
        return result
    }

    /** The Frobenius norm: `sqrt(sum of squares of every element)`. */
    fun normF() = storage.normF()

    override fun equals(other: Any?): Boolean {
        if (other !is Matrix) return false
        return storage.isIdentical(other.storage, 1e-9)
    }

    override fun hashCode() = storage.hashCode()

    override fun toString(): String = storage.toString()

    companion object {
        fun zeros(rows: Int, cols: Int) = Matrix(rows, cols)

        fun eye(n: Int) = Matrix(SimpleMatrix.identity(n))

        /** Builds a [rows]x[cols] matrix from [values], given in row-major order. */
        fun fill(rows: Int, cols: Int, vararg values: Double): Matrix {
            if (values.size != rows * cols) {
                robotPrintError("fill got ${values.size} values for a ($rows,$cols) matrix")
                return zeros(rows, cols)
            }
            val m = Matrix(rows, cols)
            for (r in 0 until rows) {
                for (c in 0 until cols) {
                    m[r, c] = values[r * cols + c]
                }
            }
            return m
        }

        /** Builds a column vector (an `n`x`1` matrix) from [values]. */
        fun vector(vararg values: Double) = fill(values.size, 1, *values)
    }
}
