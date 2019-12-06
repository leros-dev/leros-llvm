; ModuleID = 'PPath/c/ifcascademodif.c'
source_filename = "PPath/c/ifcascademodif.c"
target datalayout = "e-m:e-p:32:32-i64:64-n32-S128"
target triple = "leros32-unknown-unknown-elf"

; Function Attrs: noinline nounwind optnone
define dso_local i32 @main() #0 {
entry:
  %retval = alloca i32, align 4
  %x = alloca i32, align 4
  %y = alloca i32, align 4
  %z = alloca i32, align 4
  store i32 0, i32* %retval, align 4
  %call = call i32 @rand() #2
  store i32 %call, i32* %x, align 4
  %call1 = call i32 @rand() #2
  store i32 %call1, i32* %y, align 4
  %call2 = call i32 @rand() #2
  store i32 %call2, i32* %z, align 4
  %0 = load i32, i32* %x, align 4
  %cmp = icmp sgt i32 %0, 200
  br i1 %cmp, label %if.then, label %if.else

if.then:                                          ; preds = %entry
  %1 = load i32, i32* %x, align 4
  %inc = add nsw i32 %1, 1
  store i32 %inc, i32* %x, align 4
  br label %if.end15

if.else:                                          ; preds = %entry
  %2 = load i32, i32* %x, align 4
  %cmp3 = icmp slt i32 %2, 100
  br i1 %cmp3, label %land.lhs.true, label %if.else7

land.lhs.true:                                    ; preds = %if.else
  %3 = load i32, i32* %x, align 4
  %inc4 = add nsw i32 %3, 1
  store i32 %inc4, i32* %x, align 4
  %cmp5 = icmp sgt i32 %inc4, 50
  br i1 %cmp5, label %if.then6, label %if.else7

if.then6:                                         ; preds = %land.lhs.true
  %4 = load i32, i32* %x, align 4
  %dec = add nsw i32 %4, -1
  store i32 %dec, i32* %x, align 4
  br label %if.end14

if.else7:                                         ; preds = %land.lhs.true, %if.else
  %5 = load i32, i32* %x, align 4
  %cmp8 = icmp sle i32 %5, 50
  br i1 %cmp8, label %if.then9, label %if.else12

if.then9:                                         ; preds = %if.else7
  %6 = load i32, i32* %x, align 4
  %mul = mul nsw i32 3, %6
  %7 = load i32, i32* %x, align 4
  %add = add nsw i32 %7, %mul
  store i32 %add, i32* %x, align 4
  %8 = load i32, i32* %y, align 4
  %mul10 = mul nsw i32 2, %8
  %9 = load i32, i32* %y, align 4
  %add11 = add nsw i32 %9, %mul10
  store i32 %add11, i32* %y, align 4
  br label %if.end

if.else12:                                        ; preds = %if.else7
  %10 = load i32, i32* %x, align 4
  %add13 = add nsw i32 %10, 2
  store i32 %add13, i32* %x, align 4
  br label %if.end

if.end:                                           ; preds = %if.else12, %if.then9
  br label %if.end14

if.end14:                                         ; preds = %if.end, %if.then6
  br label %if.end15

if.end15:                                         ; preds = %if.end14, %if.then
  %11 = load i32, i32* %z, align 4
  ret i32 %11
}

; Function Attrs: nounwind
declare dso_local i32 @rand() #1

attributes #0 = { noinline nounwind optnone "correctly-rounded-divide-sqrt-fp-math"="false" "disable-tail-calls"="false" "less-precise-fpmad"="false" "min-legal-vector-width"="0" "no-frame-pointer-elim"="true" "no-frame-pointer-elim-non-leaf" "no-infs-fp-math"="false" "no-jump-tables"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="false" "stack-protector-buffer-size"="8" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #1 = { nounwind "correctly-rounded-divide-sqrt-fp-math"="false" "disable-tail-calls"="false" "less-precise-fpmad"="false" "no-frame-pointer-elim"="true" "no-frame-pointer-elim-non-leaf" "no-infs-fp-math"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="false" "stack-protector-buffer-size"="8" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #2 = { nounwind }

!llvm.module.flags = !{!0}
!llvm.ident = !{!1}

!0 = !{i32 1, !"wchar_size", i32 4}
!1 = !{!"clang version 8.0.0 (https://github.com/leros-dev/leros-clang.git ba93d76b060e90d82b2f3f3ccc6488c308790562) (https://github.com/leros-dev/leros-llvm.git d76786d5cf81a937bb119537768b22a68884ca05)"}
