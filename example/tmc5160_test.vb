Option Strict On
Option Explicit On

Imports System
Imports System.Runtime.InteropServices
Imports System.Text
Imports System.Threading

Module Tmc5160Test

    ' =========================
    ' Mechanism parameters
    ' =========================
    Private Const STEPS_PER_REV As Integer = 200
    Private Const MICROSTEPS As Integer = 256
    Private Const MM_PER_REV As Double = 4.0
    Private Const ORIGIN_MM As Double = 0.0

    ' TMC5160 internal clock
    Private Const FCLK_HZ As Integer = 12000000

    ' Position reach eps (Nothing => auto 1 ustep)
    Private POS_EPS_MM As Nullable(Of Double) = Nothing

    ' =========================
    ' Header constants
    ' =========================
    Private Const TMC_SEQ_ABSOLUTE As UInteger = 0UI
    Private Const TMC_SEQ_RELATIVE As UInteger = 1UI << 0
    Private Const TMC_SEQ_WAIT_XTOL As UInteger = 1UI << 1

    Private Const TMC_HOME_ACTIVE_LOW As UInteger = 1UI << 0
    Private Const TMC_HOME_USE_LATCH As UInteger = 1UI << 1
    Private Const TMC_HOME_SOFTSTOP As UInteger = 1UI << 2

    ' =========================
    ' DLL import
    ' =========================
    Private Const DLL_NAME As String = "tmc5160.dll"

    ' NOTE: もし落ちる/おかしい場合は CallingConvention.Cdecl を StdCall に変更
    <DllImport(DLL_NAME, CallingConvention:=CallingConvention.Cdecl, CharSet:=CharSet.Ansi)>
    Private Function tmc5160_open_index(device_index As UInteger, spi_hz As UInteger, ByRef out_h As IntPtr) As Integer
    End Function

    <DllImport(DLL_NAME, CallingConvention:=CallingConvention.Cdecl)>
    Private Sub tmc5160_close(h As IntPtr)
    End Sub

    <DllImport(DLL_NAME, CallingConvention:=CallingConvention.Cdecl)>
    Private Function tmc5160_setup_motor(h As IntPtr,
                                         ihold As Byte, irun As Byte, iholddelay As Byte,
                                         microsteps As Integer,
                                         chopconf_base As UInteger) As Integer
    End Function

    <DllImport(DLL_NAME, CallingConvention:=CallingConvention.Cdecl)>
    Private Function tmc5160_set_ramp_params(h As IntPtr,
                                             vstart As UInteger, a1 As UInteger, v1 As UInteger,
                                             amax As UInteger, vmax As UInteger, dmax As UInteger,
                                             d1 As UInteger, vstop As UInteger) As Integer
    End Function

    <DllImport(DLL_NAME, CallingConvention:=CallingConvention.Cdecl)>
    Private Function tmc5160_set_pos(h As IntPtr, pos As Integer) As Integer
    End Function

    <DllImport(DLL_NAME, CallingConvention:=CallingConvention.Cdecl)>
    Private Function tmc5160_get_pos(h As IntPtr, ByRef out_pos As Integer) As Integer
    End Function

    <DllImport(DLL_NAME, CallingConvention:=CallingConvention.Cdecl)>
    Private Function tmc5160_move_to(h As IntPtr, pos As Integer) As Integer
    End Function

    <DllImport(DLL_NAME, CallingConvention:=CallingConvention.Cdecl, CharSet:=CharSet.Ansi)>
    Private Function tmc5160_last_error_global(out_buf As StringBuilder, out_len As UInteger) As Integer
    End Function

    ' optional: dump_inputs
    <DllImport(DLL_NAME, CallingConvention:=CallingConvention.Cdecl)>
    Private Function tmc5160_dump_inputs(h As IntPtr,
                                         ByRef out_ioin As UInteger,
                                         ByRef out_rampstat As UInteger,
                                         ByRef out_sw_mode As UInteger) As Integer
    End Function

    ' optional: move_to_many
    <DllImport(DLL_NAME, CallingConvention:=CallingConvention.Cdecl)>
    Private Function tmc5160_move_to_many(h As IntPtr,
                                          positions As Integer(),
                                          count As UInteger,
                                          dwell_ms_each As UInteger,
                                          timeout_ms_each As UInteger,
                                          flags As UInteger,
                                          tol_ustep As Integer,
                                          ByRef out_done_count As UInteger) As Integer
    End Function

    ' optional: home
    <DllImport(DLL_NAME, CallingConvention:=CallingConvention.Cdecl)>
    Private Function tmc5160_home(h As IntPtr,
                                  dir As Integer,
                                  flags As UInteger,
                                  v_fast As Integer,
                                  v_slow As Integer,
                                  backoff_ustep As Integer,
                                  timeout_ms_each As UInteger,
                                  home_pos_ustep As Integer,
                                  ByRef out_latched_ustep As Integer) As Integer
    End Function

    ' optional: get_ref_pins
    <DllImport(DLL_NAME, CallingConvention:=CallingConvention.Cdecl)>
    Private Function tmc5160_get_ref_pins(h As IntPtr, ByRef out_refl As Integer, ByRef out_refr As Integer) As Integer
    End Function

    ' optional: move_to_limit
    <DllImport(DLL_NAME, CallingConvention:=CallingConvention.Cdecl)>
    Private Function tmc5160_move_to_limit(h As IntPtr, dir As Integer, velocity As Integer, timeout_ms As UInteger, ByRef out_hit_dir As Integer) As Integer
    End Function

    ' =========================
    ' mm <-> ustep conversion
    ' =========================
    Private Function UstepsPerMm() As Double
        Return (CDbl(STEPS_PER_REV) * CDbl(MICROSTEPS)) / MM_PER_REV
    End Function

    Private Function DefaultEpsMm() As Double
        Return 1.0 / UstepsPerMm()
    End Function

    Private Function MmToUstep(mm As Double) As Integer
        Return CInt(Math.Round((mm - ORIGIN_MM) * UstepsPerMm()))
    End Function

    Private Function UstepToMm(x As Integer) As Double
        Return ORIGIN_MM + (CDbl(x) / UstepsPerMm())
    End Function

    ' =========================
    ' unit conversion to TMC5160 registers
    ' =========================
    Private Function ClampU32(x As Long) As UInteger
        If x < 0 Then Return 0UI
        If x > CLng(UInteger.MaxValue) Then Return UInteger.MaxValue
        Return CUInt(x)
    End Function

    Private Function MmSToV5160(v_mm_s As Double, Optional fclk_hz As Integer = FCLK_HZ) As UInteger
        Dim v_ustep_s As Double = v_mm_s * UstepsPerMm()
        Dim v_reg As Long = CLng(Math.Round(v_ustep_s * CDbl(1 << 24) / CDbl(fclk_hz)))
        Return ClampU32(v_reg)
    End Function

    Private Function MmS2ToA5160(a_mm_s2 As Double, Optional fclk_hz As Integer = FCLK_HZ) As UInteger
        Dim a_ustep_s2 As Double = a_mm_s2 * UstepsPerMm()
        Dim denom As Double = CDbl(fclk_hz) * CDbl(fclk_hz)
        Dim a_reg As Long = CLng(Math.Round(a_ustep_s2 * (512.0 * 256.0) * (2.0 ^ 26) / denom))
        Return ClampU32(a_reg)
    End Function

    Private Function V5160ToMmS(v_reg As UInteger, Optional fclk_hz As Integer = FCLK_HZ) As Double
        Dim v_ustep_s As Double = CDbl(v_reg) * CDbl(fclk_hz) / CDbl(1 << 24)
        Return v_ustep_s / UstepsPerMm()
    End Function

    Private Function A5160ToMmS2(a_reg As UInteger, Optional fclk_hz As Integer = FCLK_HZ) As Double
        Dim num As Double = CDbl(fclk_hz) * CDbl(fclk_hz)
        Dim a_ustep_s2 As Double = CDbl(a_reg) * num / ((512.0 * 256.0) * (2.0 ^ 26))
        Return a_ustep_s2 / UstepsPerMm()
    End Function

    ' =========================
    ' helpers
    ' =========================
    Private Function LastErrGlobal() As String
        Dim sb As New StringBuilder(1024)
        tmc5160_last_error_global(sb, CUInt(sb.Capacity))
        Return sb.ToString()
    End Function

    Private Sub DumpInputs(h As IntPtr, tag As String)
        Try
            Dim ioin As UInteger = 0UI, rs As UInteger = 0UI, sw As UInteger = 0UI
            Dim rc As Integer = tmc5160_dump_inputs(h, ioin, rs, sw)

            Dim REFL As Integer = CInt((ioin >> 0) And 1UI)
            Dim REFR As Integer = CInt((ioin >> 1) And 1UI)

            Dim stopL As Integer = CInt((rs >> 0) And 1UI)
            Dim stopR As Integer = CInt((rs >> 1) And 1UI)
            Dim latchL As Integer = CInt((rs >> 2) And 1UI)
            Dim latchR As Integer = CInt((rs >> 3) And 1UI)
            Dim evL As Integer = CInt((rs >> 4) And 1UI)
            Dim evR As Integer = CInt((rs >> 5) And 1UI)
            Dim vzero As Integer = CInt((rs >> 10) And 1UI)
            Dim posR As Integer = CInt((rs >> 9) And 1UI)

            Console.WriteLine(String.Format("[dump_inputs] {0}: rc={1}  IOIN=0x{2:X8} REFL={3} REFR={4}  RAMPSTAT=0x{5:X8} stopL={6} stopR={7} latchL={8} latchR={9} evL={10} evR={11} vzero={12} posR={13}  SW_MODE=0x{14:X8}",
                                            tag, rc, ioin, REFL, REFR, rs, stopL, stopR, latchL, latchR, evL, evR, vzero, posR, sw))
        Catch ex As EntryPointNotFoundException
            ' dump_inputs not in DLL -> ignore
        End Try
    End Sub

    Private Sub CheckRc(rc As Integer, msg As String, Optional h As IntPtr = Nothing)
        If rc <> 0 Then
            If h <> IntPtr.Zero Then
                DumpInputs(h, "on_error(" & msg & ")")
            End If
            Throw New ApplicationException(msg & " rc=" & rc.ToString() & " err='" & LastErrGlobal() & "'")
        End If
    End Sub

    Private Function GetPosMm(h As IntPtr) As Double
        Dim pos As Integer = 0
        CheckRc(tmc5160_get_pos(h, pos), "get_pos", h)
        Return UstepToMm(pos)
    End Function

    Private Sub WaitReachMm(h As IntPtr, target_mm As Double, Optional timeout_s As Double = 10.0, Optional eps_mm As Nullable(Of Double) = Nothing)
        Dim eps As Double
        If eps_mm.HasValue Then
            eps = eps_mm.Value
        Else
            Dim autoEps As Double = DefaultEpsMm()
            If POS_EPS_MM.HasValue Then
                eps = Math.Max(POS_EPS_MM.Value, autoEps)
            Else
                eps = autoEps
            End If
        End If

        Dim t0 As DateTime = DateTime.UtcNow
        While True
            Dim cur_mm As Double = GetPosMm(h)
            If Math.Abs(cur_mm - target_mm) <= eps Then Return

            Dim dt As Double = (DateTime.UtcNow - t0).TotalSeconds
            If dt >= timeout_s Then
                DumpInputs(h, "wait_reach_mm timeout")
                Throw New TimeoutException("timeout: target=" & target_mm.ToString() & "mm current=" & cur_mm.ToString() & "mm eps=" & eps.ToString() & "mm")
            End If

            Thread.Sleep(5)
        End While
    End Sub

    Private Sub MoveToMm(h As IntPtr, mm As Double, Optional timeout_s As Double = 10.0, Optional eps_mm As Nullable(Of Double) = Nothing)
        Dim xt As Integer = MmToUstep(mm)
        Console.WriteLine(String.Format("move_to_mm: {0:F3} mm -> XTARGET={1}", mm, xt))
        CheckRc(tmc5160_move_to(h, xt), "move_to", h)
        WaitReachMm(h, mm, timeout_s, eps_mm)
    End Sub

    Private Sub SetRampParamsMm(h As IntPtr,
                               vstart_mm_s As Double,
                               a1_mm_s2 As Double,
                               v1_mm_s As Double,
                               amax_mm_s2 As Double,
                               vmax_mm_s As Double,
                               dmax_mm_s2 As Double,
                               d1_mm_s2 As Double,
                               vstop_mm_s As Double,
                               Optional fclk_hz As Integer = FCLK_HZ)

        Dim vstart As UInteger = MmSToV5160(vstart_mm_s, fclk_hz)
        Dim v1 As UInteger = MmSToV5160(v1_mm_s, fclk_hz)
        Dim vmax As UInteger = MmSToV5160(vmax_mm_s, fclk_hz)
        Dim vstop As UInteger = MmSToV5160(vstop_mm_s, fclk_hz)

        Dim a1 As UInteger = MmS2ToA5160(a1_mm_s2, fclk_hz)
        Dim amax As UInteger = MmS2ToA5160(amax_mm_s2, fclk_hz)
        Dim dmax As UInteger = MmS2ToA5160(dmax_mm_s2, fclk_hz)
        Dim d1 As UInteger = MmS2ToA5160(d1_mm_s2, fclk_hz)

        Console.WriteLine("[set_ramp_params_mm] mm/s, mm/s^2 => reg")
        Console.WriteLine(String.Format("  VSTART={0} ({1} mm/s)", vstart, vstart_mm_s))
        Console.WriteLine(String.Format("  V1    ={0} ({1} mm/s)", v1, v1_mm_s))
        Console.WriteLine(String.Format("  VMAX  ={0} ({1} mm/s)", vmax, vmax_mm_s))
        Console.WriteLine(String.Format("  VSTOP ={0} ({1} mm/s)", vstop, vstop_mm_s))
        Console.WriteLine(String.Format("  A1    ={0} ({1} mm/s^2)", a1, a1_mm_s2))
        Console.WriteLine(String.Format("  AMAX  ={0} ({1} mm/s^2)", amax, amax_mm_s2))
        Console.WriteLine(String.Format("  DMAX  ={0} ({1} mm/s^2)", dmax, dmax_mm_s2))
        Console.WriteLine(String.Format("  D1    ={0} ({1} mm/s^2)", d1, d1_mm_s2))
        Console.WriteLine(String.Format("  debug(back-calc) VMAX={0:F4} mm/s  AMAX={1:F4} mm/s^2", V5160ToMmS(vmax, fclk_hz), A5160ToMmS2(amax, fclk_hz)))

        CheckRc(tmc5160_set_ramp_params(h, vstart, a1, v1, amax, vmax, dmax, d1, vstop), "set_ramp_params", h)
    End Sub

    Private Function ReadRefPins(h As IntPtr) As Tuple(Of Integer, Integer)
        Try
            Dim refl As Integer = 0, refr As Integer = 0
            CheckRc(tmc5160_get_ref_pins(h, refl, refr), "get_ref_pins", h)
            Return Tuple.Create(refl, refr)
        Catch ex As EntryPointNotFoundException
            Console.WriteLine("[ref_pins] API not found in DLL")
            Return Nothing
        End Try
    End Function

    Private Function MoveToLimitMm(h As IntPtr, dir_left As Boolean, v_mm_s As Double, timeout_s As Double) As Integer
        Dim dir As Integer = If(dir_left, -1, +1)
        Dim vreg As Integer = CInt(MmSToV5160(Math.Abs(v_mm_s)))
        Dim timeout_ms As UInteger = CUInt(Math.Round(timeout_s * 1000.0))

        Dim hit As Integer = 0
        Try
            Dim lr As String = If(dir_left, "L", "R")
            DumpInputs(h, "before move_to_limit (" & lr & ")")
            Dim rc As Integer = tmc5160_move_to_limit(h, dir, vreg, timeout_ms, hit)
            CheckRc(rc, "move_to_limit", h)
            DumpInputs(h, "after move_to_limit hit=" & hit.ToString())
        Catch ex As EntryPointNotFoundException
            Throw New ApplicationException("tmc5160_move_to_limit not found in DLL")
        End Try
        Return hit
    End Function

    Private Function HomeMm(h As IntPtr,
                            dir_left As Boolean,
                            active_low As Boolean,
                            use_latch As Boolean,
                            softstop As Boolean,
                            v_fast_mm_s As Double,
                            v_slow_mm_s As Double,
                            backoff_mm As Double,
                            home_pos_mm As Double,
                            timeout_s_each As Double) As Integer

        Dim flags As UInteger = 0UI
        If active_low Then flags = flags Or TMC_HOME_ACTIVE_LOW
        If use_latch Then flags = flags Or TMC_HOME_USE_LATCH
        If softstop Then flags = flags Or TMC_HOME_SOFTSTOP

        Dim dir As Integer = If(dir_left, -1, +1)
        Dim v_fast As Integer = CInt(MmSToV5160(v_fast_mm_s))
        Dim v_slow As Integer = CInt(MmSToV5160(v_slow_mm_s))
        Dim backoff_ustep As Integer = CInt(Math.Round(Math.Abs(backoff_mm) * UstepsPerMm()))
        Dim home_pos_ustep As Integer = MmToUstep(home_pos_mm)
        Dim timeout_ms_each As UInteger = CUInt(Math.Round(timeout_s_each * 1000.0))

        Console.WriteLine(String.Format("[home_mm] dir={0} flags={1} v_fast(reg)={2} v_slow(reg)={3} backoff={4}ustep home_pos={5}ustep timeout={6}ms",
                                        dir, flags, v_fast, v_slow, backoff_ustep, home_pos_ustep, timeout_ms_each))

        DumpInputs(h, "before home")
        Dim lat As Integer = 0
        Try
            Dim rc As Integer = tmc5160_home(h, dir, flags, v_fast, v_slow, backoff_ustep, timeout_ms_each, home_pos_ustep, lat)
            CheckRc(rc, "home", h)
        Catch ex As EntryPointNotFoundException
            Throw New ApplicationException("tmc5160_home not found in DLL")
        End Try
        DumpInputs(h, "after home")
        Console.WriteLine(String.Format("[home_mm] latched={0} ustep (approx {1:F6} mm)", lat, UstepToMm(lat)))
        Return lat
    End Function

    ' =========================
    ' main
    ' =========================
    Sub Main()
        Console.WriteLine("Starting...")

        Dim h As IntPtr = IntPtr.Zero

        Try
            ' open (SPI 1MHz)
            CheckRc(tmc5160_open_index(0UI, 1000000UI, h), "open_index")
            Console.WriteLine("opened.")

            ' setup motor
            CheckRc(tmc5160_setup_motor(h, 3, 5, 6, MICROSTEPS, 0UI), "setup_motor", h)

            ' ramp params in mm/s, mm/s^2
            Dim VSTART_MM_S As Double = 0.1
            Dim VSTOP_MM_S As Double = 0.1
            Dim VMAX_MM_S As Double = 5.0
            Dim AMAX_MM_S2 As Double = 400.0

            SetRampParamsMm(h, VSTART_MM_S, AMAX_MM_S2, VMAX_MM_S, AMAX_MM_S2, VMAX_MM_S, AMAX_MM_S2, AMAX_MM_S2, VSTOP_MM_S)

            ' set current position as origin
            CheckRc(tmc5160_set_pos(h, MmToUstep(ORIGIN_MM)), "set_pos", h)

            ' ref pins
            Dim pins = ReadRefPins(h)
            If pins IsNot Nothing Then
                Console.WriteLine("REFL=" & pins.Item1.ToString() & " REFR=" & pins.Item2.ToString() & " (raw)")
            End If

            ' move to right limit
            Dim hitDir As Integer = MoveToLimitMm(h, False, 10.0, 20.0)
            Console.WriteLine("[move_to_limit_mm] hit=" & hitDir.ToString() & "  (+1=R, -1=L, 0=unknown)")

            ' homing to left
            HomeMm(h, True, True, True, False, 5.0, 1.0, 10.0, 0.0, 15.0)
            Thread.Sleep(2000)

            ' restore ramp
            SetRampParamsMm(h, VSTART_MM_S, AMAX_MM_S2, VMAX_MM_S, AMAX_MM_S2, VMAX_MM_S, AMAX_MM_S2, AMAX_MM_S2, VSTOP_MM_S)

            ' simple sequence (fallback: move_to loop)
            MoveToMm(h, -3.0, 15.0, 0.01)
            MoveToMm(h, 0.0, 15.0, 0.01)

            Console.WriteLine("done.")
        Catch ex As Exception
            Console.WriteLine("ERROR: " & ex.Message)
        Finally
            If h <> IntPtr.Zero Then
                Try
                    tmc5160_close(h)
                Catch
                End Try
            End If
        End Try
    End Sub

End Module
