//===-- LLVMOps.td - LLVM IR dialect op definition file ----*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This is the LLVM IR operation definition file.
//
//===----------------------------------------------------------------------===//

#ifndef LLVMIR_OPS
#define LLVMIR_OPS

include "mlir/Dialect/LLVMIR/LLVMOpBase.td"

class LLVM_Builder<string builder> {
  string llvmBuilder = builder;
}

def LLVM_OneResultOpBuilder : OpBuilder<
  "Builder *, OperationState &result, Type resultType, "
  "ValueRange operands, ArrayRef<NamedAttribute> attributes = {}",
  [{
    if (resultType) result.addTypes(resultType);
    result.addOperands(operands);
    for (auto namedAttr : attributes) {
      result.addAttribute(namedAttr.first, namedAttr.second);
    }
  }]>;

def LLVM_ZeroResultOpBuilder : OpBuilder<
  "Builder *, OperationState &result, ValueRange operands, "
  "ArrayRef<NamedAttribute> attributes = {}",
  [{
    result.addOperands(operands);
    for (auto namedAttr : attributes) {
      result.addAttribute(namedAttr.first, namedAttr.second);
    }
  }]>;

class LLVM_TwoBuilders<OpBuilder b1, OpBuilder b2> {
  list<OpBuilder> builders = [b1, b2];
}

// Base class for LLVM operations with one result.
class LLVM_OneResultOp<string mnemonic, list<OpTrait> traits = []> :
    LLVM_Op<mnemonic, traits>, Results<(outs LLVM_Type:$res)> {
  let builders = [LLVM_OneResultOpBuilder];
}

// Compatibility builder that takes an instance of wrapped llvm::VoidType
// to indicate no result.
def LLVM_VoidResultTypeOpBuilder : OpBuilder<
  "Builder *builder, OperationState &result, Type resultType, "
  "ValueRange operands, ArrayRef<NamedAttribute> attributes = {}",
  [{
    auto llvmType = resultType.dyn_cast<LLVM::LLVMType>(); (void)llvmType;
    assert(llvmType && "result must be an LLVM type");
    assert(llvmType.getUnderlyingType() &&
            llvmType.getUnderlyingType()->isVoidTy() &&
            "for zero-result operands, only 'void' is accepted as result type");
    build(builder, result, operands, attributes);
  }]>;

// Base class for LLVM operations with zero results.
class LLVM_ZeroResultOp<string mnemonic, list<OpTrait> traits = []> :
    LLVM_Op<mnemonic, traits>, Results<(outs)>,
    LLVM_TwoBuilders<LLVM_VoidResultTypeOpBuilder, LLVM_ZeroResultOpBuilder>;

// Base class for LLVM terminator operations.  All terminator operations have
// zero results and an optional list of successors.
class LLVM_TerminatorOp<string mnemonic, list<OpTrait> traits = []> :
    LLVM_Op<mnemonic, !listconcat(traits, [Terminator])>,
    Arguments<(ins Variadic<LLVM_Type>:$args)>, Results<(outs)> {
  let builders = [
    OpBuilder<
      "Builder *, OperationState &result, "
      "ValueRange properOperands, "
      "ArrayRef<Block *> destinations, "
      "ArrayRef<ValueRange> operands, "
      "ArrayRef<NamedAttribute> attributes = {}",
      [{
        result.addOperands(properOperands);
        for (auto kvp : llvm::zip(destinations, operands)) {
          result.addSuccessor(std::get<0>(kvp), std::get<1>(kvp));
        }
        for (auto namedAttr : attributes) {
          result.addAttribute(namedAttr.first, namedAttr.second);
        }
      }]
    >,
    OpBuilder<
      "Builder *builder, OperationState &result, "
      "ValueRange properOperands, "
      "ArrayRef<Block *> destinations, "
      "ArrayRef<NamedAttribute> attributes = {}",
    [{
        SmallVector<ValueRange, 2> operands(destinations.size(), {});
        build(builder, result, properOperands,
            destinations, operands, attributes);
      }]
    >,
  ];
}

// Class for arithmetic binary operations.
class LLVM_ArithmeticOp<string mnemonic, string builderFunc,
                        list<OpTrait> traits = []> :
    LLVM_OneResultOp<mnemonic,
           !listconcat([NoSideEffect, SameOperandsAndResultType], traits)>,
    Arguments<(ins LLVM_Type:$lhs, LLVM_Type:$rhs)>,
    LLVM_Builder<"$res = builder." # builderFunc # "($lhs, $rhs);"> {
  let parser = [{ return impl::parseOneResultSameOperandTypeOp(parser, result); }];
  let printer = [{ mlir::impl::printOneResultOp(this->getOperation(), p); }];
}
class LLVM_UnaryArithmeticOp<string mnemonic, string builderFunc,
                        list<OpTrait> traits = []> :
    LLVM_OneResultOp<mnemonic,
           !listconcat([NoSideEffect, SameOperandsAndResultType], traits)>,
    Arguments<(ins LLVM_Type:$operand)>,
    LLVM_Builder<"$res = builder." # builderFunc # "($operand);"> {
  let parser = [{ return impl::parseOneResultSameOperandTypeOp(parser, result); }];
  let printer = [{ mlir::impl::printOneResultOp(this->getOperation(), p); }];
}

// Integer binary operations.
def LLVM_AddOp : LLVM_ArithmeticOp<"add", "CreateAdd", [Commutative]>;
def LLVM_SubOp : LLVM_ArithmeticOp<"sub", "CreateSub">;
def LLVM_MulOp : LLVM_ArithmeticOp<"mul", "CreateMul", [Commutative]>;
def LLVM_UDivOp : LLVM_ArithmeticOp<"udiv", "CreateUDiv">;
def LLVM_SDivOp : LLVM_ArithmeticOp<"sdiv", "CreateSDiv">;
def LLVM_URemOp : LLVM_ArithmeticOp<"urem", "CreateURem">;
def LLVM_SRemOp : LLVM_ArithmeticOp<"srem", "CreateSRem">;
def LLVM_AndOp : LLVM_ArithmeticOp<"and", "CreateAnd">;
def LLVM_OrOp : LLVM_ArithmeticOp<"or", "CreateOr">;
def LLVM_XOrOp : LLVM_ArithmeticOp<"xor", "CreateXor">;
def LLVM_ShlOp : LLVM_ArithmeticOp<"shl", "CreateShl">;
def LLVM_LShrOp : LLVM_ArithmeticOp<"lshr", "CreateLShr">;
def LLVM_AShrOp : LLVM_ArithmeticOp<"ashr", "CreateAShr">;

// Predicate for integer comparisons.
def ICmpPredicateEQ  : I64EnumAttrCase<"eq", 0>;
def ICmpPredicateNE  : I64EnumAttrCase<"ne", 1>;
def ICmpPredicateSLT : I64EnumAttrCase<"slt", 2>;
def ICmpPredicateSLE : I64EnumAttrCase<"sle", 3>;
def ICmpPredicateSGT : I64EnumAttrCase<"sgt", 4>;
def ICmpPredicateSGE : I64EnumAttrCase<"sge", 5>;
def ICmpPredicateULT : I64EnumAttrCase<"ult", 6>;
def ICmpPredicateULE : I64EnumAttrCase<"ule", 7>;
def ICmpPredicateUGT : I64EnumAttrCase<"ugt", 8>;
def ICmpPredicateUGE : I64EnumAttrCase<"uge", 9>;
def ICmpPredicate : I64EnumAttr<
    "ICmpPredicate",
    "llvm.icmp comparison predicate",
    [ICmpPredicateEQ, ICmpPredicateNE, ICmpPredicateSLT, ICmpPredicateSLE,
     ICmpPredicateSGT, ICmpPredicateSGE, ICmpPredicateULT, ICmpPredicateULE,
     ICmpPredicateUGT, ICmpPredicateUGE]> {
  let cppNamespace = "::mlir::LLVM";
}

// Other integer operations.
def LLVM_ICmpOp : LLVM_OneResultOp<"icmp", [NoSideEffect]>,
                  Arguments<(ins ICmpPredicate:$predicate, LLVM_Type:$lhs,
                             LLVM_Type:$rhs)> {
  let llvmBuilder = [{
    $res = builder.CreateICmp(getLLVMCmpPredicate($predicate), $lhs, $rhs);
  }];
  let builders = [OpBuilder<
    "Builder *b, OperationState &result, ICmpPredicate predicate, Value lhs, "
    "Value rhs", [{
      LLVMDialect *dialect = &lhs.getType().cast<LLVMType>().getDialect();
      build(b, result, LLVMType::getInt1Ty(dialect),
            b->getI64IntegerAttr(static_cast<int64_t>(predicate)), lhs, rhs);
    }]>];
  let parser = [{ return parseCmpOp<ICmpPredicate>(parser, result); }];
  let printer = [{ printICmpOp(p, *this); }];
}

// Predicate for float comparisons
def FCmpPredicateFALSE  : I64EnumAttrCase<"_false", 0>;
def FCmpPredicateOEQ    : I64EnumAttrCase<"oeq", 1>;
def FCmpPredicateOGT    : I64EnumAttrCase<"ogt", 2>;
def FCmpPredicateOGE    : I64EnumAttrCase<"oge", 3>;
def FCmpPredicateOLT    : I64EnumAttrCase<"olt", 4>;
def FCmpPredicateOLE    : I64EnumAttrCase<"ole", 5>;
def FCmpPredicateONE    : I64EnumAttrCase<"one", 6>;
def FCmpPredicateORD    : I64EnumAttrCase<"ord", 7>;
def FCmpPredicateUEQ    : I64EnumAttrCase<"ueq", 8>;
def FCmpPredicateUGT    : I64EnumAttrCase<"ugt", 9>;
def FCmpPredicateUGE    : I64EnumAttrCase<"uge", 10>;
def FCmpPredicateULT    : I64EnumAttrCase<"ult", 11>;
def FCmpPredicateULE    : I64EnumAttrCase<"ule", 12>;
def FCmpPredicateUNE    : I64EnumAttrCase<"une", 13>;
def FCmpPredicateUNO    : I64EnumAttrCase<"uno", 14>;
def FCmpPredicateTRUE   : I64EnumAttrCase<"_true", 15>;

def FCmpPredicate : I64EnumAttr<
    "FCmpPredicate",
    "llvm.fcmp comparison predicate",
    [FCmpPredicateFALSE, FCmpPredicateOEQ, FCmpPredicateOGT, FCmpPredicateOGE,
     FCmpPredicateOLT, FCmpPredicateOLE, FCmpPredicateONE, FCmpPredicateORD,
     FCmpPredicateUEQ, FCmpPredicateUGT, FCmpPredicateUGE, FCmpPredicateULT,
     FCmpPredicateULE, FCmpPredicateUNE, FCmpPredicateUNO, FCmpPredicateTRUE
    ]> {
  let cppNamespace = "::mlir::LLVM";
}

// Other integer operations.
def LLVM_FCmpOp : LLVM_OneResultOp<"fcmp", [NoSideEffect]>,
                  Arguments<(ins FCmpPredicate:$predicate, LLVM_Type:$lhs,
                             LLVM_Type:$rhs)> {
  let llvmBuilder = [{
    $res = builder.CreateFCmp(getLLVMCmpPredicate($predicate), $lhs, $rhs);
  }];
  let builders = [OpBuilder<
    "Builder *b, OperationState &result, FCmpPredicate predicate, Value lhs, "
    "Value rhs", [{
      LLVMDialect *dialect = &lhs.getType().cast<LLVMType>().getDialect();
      build(b, result, LLVMType::getInt1Ty(dialect),
            b->getI64IntegerAttr(static_cast<int64_t>(predicate)), lhs, rhs);
    }]>];
  let parser = [{ return parseCmpOp<FCmpPredicate>(parser, result); }];
  let printer = [{ printFCmpOp(p, *this); }];
}

// Floating point binary operations.
def LLVM_FAddOp : LLVM_ArithmeticOp<"fadd", "CreateFAdd">;
def LLVM_FSubOp : LLVM_ArithmeticOp<"fsub", "CreateFSub">;
def LLVM_FMulOp : LLVM_ArithmeticOp<"fmul", "CreateFMul">;
def LLVM_FDivOp : LLVM_ArithmeticOp<"fdiv", "CreateFDiv">;
def LLVM_FRemOp : LLVM_ArithmeticOp<"frem", "CreateFRem">;
def LLVM_FNegOp : LLVM_UnaryArithmeticOp<"fneg", "CreateFNeg">;

// Memory-related operations.
def LLVM_AllocaOp :
    LLVM_OneResultOp<"alloca">,
    Arguments<(ins LLVM_Type:$arraySize, OptionalAttr<I64Attr>:$alignment)> {
  string llvmBuilder = [{
    auto *alloca = builder.CreateAlloca(
      $_resultType->getPointerElementType(), $arraySize);
    if ($alignment.hasValue()) {
      auto align = $alignment.getValue().getZExtValue();
      if (align != 0)
        alloca->setAlignment(llvm::MaybeAlign(align));
    }
    $res = alloca;
  }];
  let builders = [OpBuilder<
    "Builder *b, OperationState &result, Type resultType, Value arraySize, "
    "unsigned alignment",
    [{
      if (alignment == 0)
        return build(b, result, resultType, arraySize, IntegerAttr());
      build(b, result, resultType, arraySize, b->getI64IntegerAttr(alignment));
  }]>];
  let parser = [{ return parseAllocaOp(parser, result); }];
  let printer = [{ printAllocaOp(p, *this); }];
  let verifier = [{
    if (alignment().hasValue()) {
      auto align = alignment().getValue().getSExtValue();
      if (align < 0)
        return emitOpError("expected positive alignment");
    }
    return success();
  }];
}
def LLVM_GEPOp : LLVM_OneResultOp<"getelementptr", [NoSideEffect]>,
                 Arguments<(ins LLVM_Type:$base, Variadic<LLVM_Type>:$indices)>,
                 LLVM_Builder<"$res = builder.CreateGEP($base, $indices);"> {
  let assemblyFormat = [{
    $base `[` $indices `]` attr-dict `:` functional-type(operands, results)
  }];
}
def LLVM_LoadOp : LLVM_OneResultOp<"load">, Arguments<(ins LLVM_Type:$addr)>,
                  LLVM_Builder<"$res = builder.CreateLoad($addr);"> {
  let builders = [OpBuilder<
    "Builder *b, OperationState &result, Value addr",
    [{
      auto type = addr.getType().cast<LLVM::LLVMType>().getPointerElementTy();
      build(b, result, type, addr);
    }]>];
  let parser = [{ return parseLoadOp(parser, result); }];
  let printer = [{ printLoadOp(p, *this); }];
}
def LLVM_StoreOp : LLVM_ZeroResultOp<"store">,
                   Arguments<(ins LLVM_Type:$value, LLVM_Type:$addr)>,
                   LLVM_Builder<"builder.CreateStore($value, $addr);"> {
  let parser = [{ return parseStoreOp(parser, result); }];
  let printer = [{ printStoreOp(p, *this); }];
}

// Casts.
class LLVM_CastOp<string mnemonic, string builderFunc,
                  list<OpTrait> traits = []> :
    LLVM_OneResultOp<mnemonic,
           !listconcat([NoSideEffect], traits)>,
    Arguments<(ins LLVM_Type:$arg)>,
    LLVM_Builder<"$res = builder." # builderFunc # "($arg, $_resultType);"> {
  let parser = [{ return mlir::impl::parseCastOp(parser, result); }];
  let printer = [{ mlir::impl::printCastOp(this->getOperation(), p); }];
}
def LLVM_BitcastOp : LLVM_CastOp<"bitcast", "CreateBitCast">;
def LLVM_AddrSpaceCastOp : LLVM_CastOp<"addrspacecast", "CreateAddrSpaceCast">;
def LLVM_IntToPtrOp : LLVM_CastOp<"inttoptr", "CreateIntToPtr">;
def LLVM_PtrToIntOp : LLVM_CastOp<"ptrtoint", "CreatePtrToInt">;
def LLVM_SExtOp : LLVM_CastOp<"sext", "CreateSExt">;
def LLVM_ZExtOp : LLVM_CastOp<"zext", "CreateZExt">;
def LLVM_TruncOp : LLVM_CastOp<"trunc", "CreateTrunc">;
def LLVM_SIToFPOp : LLVM_CastOp<"sitofp", "CreateSIToFP">;
def LLVM_UIToFPOp : LLVM_CastOp<"uitofp", "CreateUIToFP">;
def LLVM_FPToSIOp : LLVM_CastOp<"fptosi", "CreateFPToSI">;
def LLVM_FPToUIOp : LLVM_CastOp<"fptoui", "CreateFPToUI">;
def LLVM_FPExtOp : LLVM_CastOp<"fpext", "CreateFPExt">;
def LLVM_FPTruncOp : LLVM_CastOp<"fptrunc", "CreateFPTrunc">;

// Call-related operations.
def LLVM_InvokeOp : LLVM_Op<"invoke", [Terminator]>,
                    Arguments<(ins OptionalAttr<FlatSymbolRefAttr>:$callee,
                               Variadic<LLVM_Type>)>,
                    Results<(outs Variadic<LLVM_Type>)> {
  let builders = [OpBuilder<
    "Builder *b, OperationState &result, ArrayRef<Type> tys, "
    "FlatSymbolRefAttr callee, ValueRange ops, Block* normal, "
    "ValueRange normalOps, Block* unwind, ValueRange unwindOps",
    [{
      result.addAttribute("callee", callee);
      build(b, result, tys, ops, normal, normalOps, unwind, unwindOps);
    }]>,
    OpBuilder<
    "Builder *b, OperationState &result, ArrayRef<Type> tys, "
    "ValueRange ops, Block* normal, "
    "ValueRange normalOps, Block* unwind, ValueRange unwindOps",
    [{
      result.addTypes(tys);
      result.addOperands(ops);
      result.addSuccessor(normal, normalOps);
      result.addSuccessor(unwind, unwindOps);
    }]>];
  let verifier = [{ return ::verify(*this);  }];
  let parser = [{ return parseInvokeOp(parser, result); }];
  let printer = [{ printInvokeOp(p, *this); }];
}

def LLVM_LandingpadOp : LLVM_OneResultOp<"landingpad">,
                        Arguments<(ins UnitAttr:$cleanup, 
                                       Variadic<LLVM_Type>)> {
  let verifier = [{ return ::verify(*this); }];
  let parser = [{ return parseLandingpadOp(parser, result); }];
  let printer = [{ printLandingpadOp(p, *this); }];
}

def LLVM_CallOp : LLVM_Op<"call">,
                  Arguments<(ins OptionalAttr<FlatSymbolRefAttr>:$callee,
                             Variadic<LLVM_Type>)>,
                  Results<(outs Variadic<LLVM_Type>)>,
                  LLVM_TwoBuilders<LLVM_OneResultOpBuilder,
                                   LLVM_ZeroResultOpBuilder> {
  let verifier = [{
    if (getNumResults() > 1)
      return emitOpError("must have 0 or 1 result");
    return success();
  }];
  let parser = [{ return parseCallOp(parser, result); }];
  let printer = [{ printCallOp(p, *this); }];
}
def LLVM_ExtractElementOp : LLVM_OneResultOp<"extractelement", [NoSideEffect]>,
                          Arguments<(ins LLVM_Type:$vector,
                                     LLVM_Type:$position)> {
  string llvmBuilder = [{
    $res = builder.CreateExtractElement($vector, $position);
  }];
  let builders = [OpBuilder<
    "Builder *b, OperationState &result, Value vector, Value position,"
    "ArrayRef<NamedAttribute> attrs = {}">];
  let parser = [{ return parseExtractElementOp(parser, result); }];
  let printer = [{ printExtractElementOp(p, *this); }];
}
def LLVM_ExtractValueOp : LLVM_OneResultOp<"extractvalue", [NoSideEffect]>,
                          Arguments<(ins LLVM_Type:$container,
                                     ArrayAttr:$position)> {
  string llvmBuilder = [{
    $res = builder.CreateExtractValue($container, extractPosition($position));
  }];
  let parser = [{ return parseExtractValueOp(parser, result); }];
  let printer = [{ printExtractValueOp(p, *this); }];
}
def LLVM_InsertElementOp : LLVM_OneResultOp<"insertelement", [NoSideEffect]>,
                         Arguments<(ins LLVM_Type:$vector, LLVM_Type:$value,
                                    LLVM_Type:$position)> {
  string llvmBuilder = [{
    $res = builder.CreateInsertElement($vector, $value, $position);
  }];
  let parser = [{ return parseInsertElementOp(parser, result); }];
  let printer = [{ printInsertElementOp(p, *this); }];
}
def LLVM_InsertValueOp : LLVM_OneResultOp<"insertvalue", [NoSideEffect]>,
                         Arguments<(ins LLVM_Type:$container, LLVM_Type:$value,
                                    ArrayAttr:$position)> {
  string llvmBuilder = [{
    $res = builder.CreateInsertValue($container, $value,
                                     extractPosition($position));
  }];
  let builders = [OpBuilder<
    "Builder *b, OperationState &result, Value container, Value value, "
    "ArrayAttr position",
    [{
      build(b, result, container.getType(), container, value, position);
    }]>];
  let parser = [{ return parseInsertValueOp(parser, result); }];
  let printer = [{ printInsertValueOp(p, *this); }];
}
def LLVM_ShuffleVectorOp
    : LLVM_OneResultOp<"shufflevector", [NoSideEffect]>,
      Arguments<(ins LLVM_Type:$v1, LLVM_Type:$v2, ArrayAttr:$mask)>,
      LLVM_Builder<
      "$res = builder.CreateShuffleVector($v1, $v2, extractPosition($mask));"> {
  let builders = [OpBuilder<
    "Builder *b, OperationState &result, Value v1, Value v2, "
    "ArrayAttr mask, ArrayRef<NamedAttribute> attrs = {}">];
  let verifier = [{
    auto wrappedVectorType1 = v1().getType().cast<LLVM::LLVMType>();
    auto wrappedVectorType2 = v2().getType().cast<LLVM::LLVMType>();
    if (!wrappedVectorType2.getUnderlyingType()->isVectorTy())
      return emitOpError("expected LLVM IR Dialect vector type for operand #2");
    if (wrappedVectorType1.getVectorElementType() !=
        wrappedVectorType2.getVectorElementType())
      return emitOpError("expected matching LLVM IR Dialect element types");
    return success();
  }];
  let parser = [{ return parseShuffleVectorOp(parser, result); }];
  let printer = [{ printShuffleVectorOp(p, *this); }];
}

// Misc operations.
def LLVM_SelectOp
    : LLVM_OneResultOp<"select", [NoSideEffect]>,
      Arguments<(ins LLVM_Type:$condition, LLVM_Type:$trueValue,
                 LLVM_Type:$falseValue)>,
      LLVM_Builder<
          "$res = builder.CreateSelect($condition, $trueValue, $falseValue);"> {
  let builders = [OpBuilder<
    "Builder *b, OperationState &result, Value condition, Value lhs, "
    "Value rhs", [{
      build(b, result, lhs.getType(), condition, lhs, rhs);
    }]>];
  let parser = [{ return parseSelectOp(parser, result); }];
  let printer = [{ printSelectOp(p, *this); }];
}

// Terminators.
def LLVM_BrOp : LLVM_TerminatorOp<"br", []> {
  let parser = [{ return parseBrOp(parser, result); }];
  let printer = [{ printBrOp(p, *this); }];
}
def LLVM_CondBrOp : LLVM_TerminatorOp<"cond_br", []> {
  let verifier = [{
    if (getNumSuccessors() != 2)
      return emitOpError("expected exactly two successors");
    return success();
  }];
  let parser = [{ return parseCondBrOp(parser, result); }];
  let printer = [{ printCondBrOp(p, *this); }];
}
def LLVM_ReturnOp : LLVM_TerminatorOp<"return", []> {
  string llvmBuilder = [{
    if ($_numOperands != 0)
      builder.CreateRet($args[0]);
    else
      builder.CreateRetVoid();
  }];

  let verifier = [{
    if (getNumOperands() > 1)
      return emitOpError("expects at most 1 operand");
    return success();
  }];

  let parser = [{ return parseReturnOp(parser, result); }];
  let printer = [{ printReturnOp(p, *this); }];
}
def LLVM_UnreachableOp : LLVM_TerminatorOp<"unreachable", []> {
  string llvmBuilder = [{ builder.CreateUnreachable(); }];
  let parser = [{ return success(); }];
  let printer = [{ p << getOperationName(); }];
}

////////////////////////////////////////////////////////////////////////////////
// Auxiliary operations (do not appear in LLVM IR but necessary for the dialect
// to work correctly).
////////////////////////////////////////////////////////////////////////////////

// Linkage attribute is used on functions and globals. The order follows that of
// https://llvm.org/docs/LangRef.html#linkage-types. The names are equivalent to
// visible names in the IR rather than to enum values names in llvm::GlobalValue
// since the latter is easier to change.
def LinkagePrivate
    : LLVM_EnumAttrCase<"Private", "private", "PrivateLinkage", 0>;
def LinkageInternal
    : LLVM_EnumAttrCase<"Internal", "internal", "InternalLinkage", 1>;
def LinkageAvailableExternally
    : LLVM_EnumAttrCase<"AvailableExternally", "available_externally",
                        "AvailableExternallyLinkage", 2>;
def LinkageLinkonce
    : LLVM_EnumAttrCase<"Linkonce", "linkonce", "LinkOnceAnyLinkage", 3>;
def LinkageWeak
    : LLVM_EnumAttrCase<"Weak", "weak", "WeakAnyLinkage", 4>;
def LinkageCommon
    : LLVM_EnumAttrCase<"Common", "common", "CommonLinkage", 5>;
def LinkageAppending
    : LLVM_EnumAttrCase<"Appending", "appending", "AppendingLinkage", 6>;
def LinkageExternWeak
   : LLVM_EnumAttrCase<"ExternWeak", "extern_weak", "ExternalWeakLinkage", 7>;
def LinkageLinkonceODR
    : LLVM_EnumAttrCase<"LinkonceODR", "linkonce_odr", "LinkOnceODRLinkage", 8>;
def LinkageWeakODR
    : LLVM_EnumAttrCase<"WeakODR", "weak_odr", "WeakODRLinkage", 9>;
def LinkageExternal
    : LLVM_EnumAttrCase<"External", "external", "ExternalLinkage", 10>;

def Linkage : LLVM_EnumAttr<
    "Linkage",
    "::llvm::GlobalValue::LinkageTypes",
    "LLVM linkage types",
    [LinkagePrivate, LinkageInternal, LinkageAvailableExternally,
     LinkageLinkonce, LinkageWeak, LinkageCommon, LinkageAppending,
     LinkageExternWeak, LinkageLinkonceODR, LinkageWeakODR, LinkageExternal]> {
  let cppNamespace = "::mlir::LLVM";
}


def LLVM_AddressOfOp
    : LLVM_OneResultOp<"mlir.addressof">,
      Arguments<(ins FlatSymbolRefAttr:$global_name)> {
  let builders = [
    OpBuilder<"Builder *builder, OperationState &result, LLVMType resType, "
              "StringRef name, ArrayRef<NamedAttribute> attrs = {}", [{
      result.addAttribute("global_name", builder->getSymbolRefAttr(name));
      result.addAttributes(attrs);
      result.addTypes(resType);}]>,

    OpBuilder<"Builder *builder, OperationState &result, GlobalOp global, "
              "ArrayRef<NamedAttribute> attrs = {}", [{
      build(builder, result,
            global.getType().getPointerTo(global.addr_space().getZExtValue()),
            global.sym_name(), attrs);}]>
  ];

  let extraClassDeclaration = [{
    /// Return the llvm.mlir.global operation that defined the value referenced
    /// here.
    GlobalOp getGlobal();
  }];

  let assemblyFormat = "$global_name attr-dict `:` type($res)";
  let verifier = "return ::verify(*this);";
}

def LLVM_GlobalOp
    : LLVM_ZeroResultOp<"mlir.global",
                        [IsolatedFromAbove,
                         SingleBlockImplicitTerminator<"ReturnOp">, Symbol]>,
      Arguments<(ins TypeAttr:$type, UnitAttr:$constant, StrAttr:$sym_name,
                 Linkage:$linkage,
                 OptionalAttr<AnyAttr>:$value,
                 DefaultValuedAttr<NonNegativeI32Attr, "0">:$addr_space)> {
  let summary = "LLVM dialect global.";
  let description = [{
    Can contain an optional initializer region or attribute for simple
    initializers.

    Examples:
      // Initialized using an attribute.
      llvm.mlir.global @a("abc") : !llvm<"[3 x i8]">
      // Initialized using a region.
      llvm.mlir.global constant @b() : !llvm<"i32*"> {
        %0 = llvm.constant(0 : i32) : !llvm.i32
        %1 = llvm.inttoptr %0 : !llvm.i32 to !llvm<"i32*">
        llvm.return %1 : !llvm<"i32*">
      }
  }];
  let regions = (region AnyRegion:$initializer);

  let builders = [
    OpBuilder<"Builder *builder, OperationState &result, LLVMType type, "
              "bool isConstant, Linkage linkage, StringRef name, "
              "Attribute value, unsigned addrSpace = 0, "
              "ArrayRef<NamedAttribute> attrs = {}">
  ];

  let extraClassDeclaration = [{
    /// Return the LLVM type of the global.
    LLVMType getType() {
      return type().cast<LLVMType>();
    }
    /// Return the initializer attribute if it exists, or a null attribute.
    Attribute getValueOrNull() {
      return value().getValueOr(Attribute());
    }
    /// Return the initializer region. This may be empty, but if it is not it
    /// terminates in an `llvm.return` op with the initializer value.
    Region &getInitializerRegion() {
      return getOperation()->getRegion(0);
    }
    /// Return the initializer block. If the initializer region is empty this
    /// is nullptr. If it is not nullptr, it terminates with an `llvm.return`
    /// op with the initializer value.
    Block *getInitializerBlock() {
      return getInitializerRegion().empty() ?
        nullptr : &getInitializerRegion().front();
    }
  }];

  let printer = "printGlobalOp(p, *this);";
  let parser = "return parseGlobalOp(parser, result);";
  let verifier = "return ::verify(*this);";
}

def LLVM_LLVMFuncOp
    : LLVM_ZeroResultOp<"func", [IsolatedFromAbove, FunctionLike, Symbol]>,
      Arguments<(ins DefaultValuedAttr<Linkage,
                                       "Linkage::External">:$linkage)> {
  let summary = "LLVM dialect function, has wrapped LLVM IR function type";

  let regions = (region AnyRegion:$body);

  let skipDefaultBuilders = 1;

  let builders = [
    OpBuilder<"Builder *builder, OperationState &result, StringRef name, "
              "LLVMType type, LLVM::Linkage linkage = LLVM::Linkage::External, "
              "ArrayRef<NamedAttribute> attrs = {}, "
              "ArrayRef<NamedAttributeList> argAttrs = {}">
  ];

  let extraClassDeclaration = [{
    // Add an entry block to an empty function, and set up the block arguments
    // to match the signature of the function.
    Block *addEntryBlock();

    LLVMType getType() {
      return getAttrOfType<TypeAttr>(getTypeAttrName())
          .getValue().cast<LLVMType>();
    }
    bool isVarArg() {
      return getType().getUnderlyingType()->isFunctionVarArg();
    }

    // Hook for OpTrait::FunctionLike, returns the number of function arguments.
    // Depends on the type attribute being correct as checked by verifyType.
    unsigned getNumFuncArguments();

    // Hook for OpTrait::FunctionLike, returns the number of function results.
    // Depends on the type attribute being correct as checked by verifyType.
    unsigned getNumFuncResults();

    // Hook for OpTrait::FunctionLike, called after verifying that the 'type'
    // attribute is present.  This can check for preconditions of the
    // getNumArguments hook not failing.
    LogicalResult verifyType();
  }];

  let verifier = [{ return ::verify(*this); }];
  let printer = [{ printLLVMFuncOp(p, *this); }];
  let parser = [{ return parseLLVMFuncOp(parser, result); }];
}

def LLVM_NullOp
    : LLVM_OneResultOp<"mlir.null", [NoSideEffect]>,
      LLVM_Builder<"$res = llvm::ConstantPointerNull::get("
                   "    cast<llvm::PointerType>($_resultType));"> {
  let assemblyFormat = "attr-dict `:` type($res)";
  let verifier = [{ return ::verify(*this); }];
}

def LLVM_UndefOp : LLVM_OneResultOp<"mlir.undef", [NoSideEffect]>,
                   LLVM_Builder<"$res = llvm::UndefValue::get($_resultType);"> {
  let assemblyFormat = "attr-dict `:` type($res)";
}
def LLVM_ConstantOp
    : LLVM_OneResultOp<"mlir.constant", [NoSideEffect]>,
      Arguments<(ins AnyAttr:$value)>,
      LLVM_Builder<"$res = getLLVMConstant($_resultType, $value, $_location);">
{
  let assemblyFormat = "`(` $value `)` attr-dict `:` type($res)";
}

// Operations that correspond to LLVM intrinsics. With MLIR operation set being
// extendable, there is no reason to introduce a hard boundary between "core"
// operations and intrinsics. However, we systematically prefix them with
// "intr." to avoid potential name clashes.

class LLVM_UnaryIntrinsicOp<string func, list<OpTrait> traits = []> :
    LLVM_OneResultOp<"intr." # func,
           !listconcat([NoSideEffect, SameOperandsAndResultType], traits)>,
    Arguments<(ins LLVM_Type:$in)>,
    LLVM_Builder<"$res = builder.CreateCall(llvm::Intrinsic::getDeclaration("
      "builder.GetInsertBlock()->getModule(), llvm::Intrinsic::" # func # ","
      "{$in->getType()}), {$in});"> {
}

class LLVM_BinarySameArgsIntrinsicOp<string func, list<OpTrait> traits = []> :
    LLVM_OneResultOp<"intr." # func,
           !listconcat([NoSideEffect, SameOperandsAndResultType], traits)>,
    Arguments<(ins LLVM_Type:$a, LLVM_Type:$b)>,
    LLVM_Builder<"$res = builder.CreateCall(llvm::Intrinsic::getDeclaration("
      "builder.GetInsertBlock()->getModule(), llvm::Intrinsic::" # func # ","
      "{$a->getType()}), {$a, $b});"> {
}

class LLVM_TernarySameArgsIntrinsicOp<string func, list<OpTrait> traits = []> :
    LLVM_OneResultOp<"intr." # func,
           !listconcat([NoSideEffect, SameOperandsAndResultType], traits)>,
    Arguments<(ins LLVM_Type:$a, LLVM_Type:$b, LLVM_Type:$c)>,
    LLVM_Builder<"$res = builder.CreateCall(llvm::Intrinsic::getDeclaration("
      "builder.GetInsertBlock()->getModule(), llvm::Intrinsic::" # func # ","
      "{$a->getType()}), {$a, $b, $c});"> {
}

def LLVM_ExpOp : LLVM_UnaryIntrinsicOp<"exp">;
def LLVM_FAbsOp : LLVM_UnaryIntrinsicOp<"fabs">;
def LLVM_FCeilOp : LLVM_UnaryIntrinsicOp<"ceil">;
def LLVM_CosOp : LLVM_UnaryIntrinsicOp<"cos">;
def LLVM_CopySignOp : LLVM_BinarySameArgsIntrinsicOp<"copysign">;
def LLVM_FMulAddOp : LLVM_TernarySameArgsIntrinsicOp<"fmuladd">;
def LLVM_SqrtOp : LLVM_UnaryIntrinsicOp<"sqrt">;

def LLVM_LogOp : LLVM_Op<"intr.log", [NoSideEffect]>,
                   Arguments<(ins LLVM_Type:$in)>,
                   Results<(outs LLVM_Type:$res)> {
  let llvmBuilder = [{
    llvm::Module *module = builder.GetInsertBlock()->getModule();
    llvm::Function *fn = llvm::Intrinsic::getDeclaration(
        module, llvm::Intrinsic::log, {$in->getType()});
    $res = builder.CreateCall(fn, {$in});
  }];
}

def LLVM_Log10Op : LLVM_Op<"intr.log10", [NoSideEffect]>,
                   Arguments<(ins LLVM_Type:$in)>,
                   Results<(outs LLVM_Type:$res)> {
  let llvmBuilder = [{
    llvm::Module *module = builder.GetInsertBlock()->getModule();
    llvm::Function *fn = llvm::Intrinsic::getDeclaration(
        module, llvm::Intrinsic::log10, {$in->getType()});
    $res = builder.CreateCall(fn, {$in});
  }];
}

def LLVM_Log2Op : LLVM_Op<"intr.log2", [NoSideEffect]>,
                   Arguments<(ins LLVM_Type:$in)>,
                   Results<(outs LLVM_Type:$res)> {
  let llvmBuilder = [{
    llvm::Module *module = builder.GetInsertBlock()->getModule();
    llvm::Function *fn = llvm::Intrinsic::getDeclaration(
        module, llvm::Intrinsic::log2, {$in->getType()});
    $res = builder.CreateCall(fn, {$in});
  }];
}

def LLVM_Prefetch : LLVM_ZeroResultOp<"intr.prefetch">,
                    Arguments<(ins LLVM_Type:$addr, LLVM_Type:$rw,
                    LLVM_Type:$hint, LLVM_Type:$cache)> {
  let llvmBuilder = [{
    llvm::Module *module = builder.GetInsertBlock()->getModule();
    llvm::Function *fn = llvm::Intrinsic::getDeclaration(
        module, llvm::Intrinsic::prefetch, $addr->getType());
    builder.CreateCall(fn, {$addr, $rw, $hint, $cache});
  }];
}

def AtomicBinOpXchg : I64EnumAttrCase<"xchg", 0>;
def AtomicBinOpAdd  : I64EnumAttrCase<"add", 1>;
def AtomicBinOpSub  : I64EnumAttrCase<"sub", 2>;
def AtomicBinOpAnd  : I64EnumAttrCase<"_and", 3>;
def AtomicBinOpNand : I64EnumAttrCase<"nand", 4>;
def AtomicBinOpOr   : I64EnumAttrCase<"_or", 5>;
def AtomicBinOpXor  : I64EnumAttrCase<"_xor", 6>;
def AtomicBinOpMax  : I64EnumAttrCase<"max", 7>;
def AtomicBinOpMin  : I64EnumAttrCase<"min", 8>;
def AtomicBinOpUMax : I64EnumAttrCase<"umax", 9>;
def AtomicBinOpUMin : I64EnumAttrCase<"umin", 10>;
def AtomicBinOpFAdd : I64EnumAttrCase<"fadd", 11>;
def AtomicBinOpFSub : I64EnumAttrCase<"fsub", 12>;
def AtomicBinOp : I64EnumAttr<
    "AtomicBinOp",
    "llvm.atomicrmw binary operations",
    [AtomicBinOpXchg, AtomicBinOpAdd, AtomicBinOpSub, AtomicBinOpAnd,
     AtomicBinOpNand, AtomicBinOpOr, AtomicBinOpXor, AtomicBinOpMax,
     AtomicBinOpMin, AtomicBinOpUMax, AtomicBinOpUMin, AtomicBinOpFAdd,
     AtomicBinOpFSub]> {
  let cppNamespace = "::mlir::LLVM";
}

def AtomicOrderingNotAtomic              : I64EnumAttrCase<"not_atomic", 0>;
def AtomicOrderingUnordered              : I64EnumAttrCase<"unordered", 1>;
def AtomicOrderingMonotonic              : I64EnumAttrCase<"monotonic", 2>;
def AtomicOrderingAcquire                : I64EnumAttrCase<"acquire", 4>;
def AtomicOrderingRelease                : I64EnumAttrCase<"release", 5>;
def AtomicOrderingAcquireRelease         : I64EnumAttrCase<"acq_rel", 6>;
def AtomicOrderingSequentiallyConsistent : I64EnumAttrCase<"seq_cst", 7>;
def AtomicOrdering : I64EnumAttr<
    "AtomicOrdering",
    "Atomic ordering for LLVM's memory model",
    [AtomicOrderingNotAtomic, AtomicOrderingUnordered, AtomicOrderingMonotonic,
     AtomicOrderingAcquire, AtomicOrderingRelease, AtomicOrderingAcquireRelease,
     AtomicOrderingSequentiallyConsistent]> {
  let cppNamespace = "::mlir::LLVM";
}

def LLVM_AtomicRMWOp : LLVM_Op<"atomicrmw">,
    Arguments<(ins AtomicBinOp:$bin_op, LLVM_Type:$ptr, LLVM_Type:$val,
                   AtomicOrdering:$ordering)>,
    Results<(outs LLVM_Type:$res)> {
  let llvmBuilder = [{
    $res = builder.CreateAtomicRMW(getLLVMAtomicBinOp($bin_op), $ptr, $val,
                                   getLLVMAtomicOrdering($ordering));
  }];
  let parser = [{ return parseAtomicRMWOp(parser, result); }];
  let printer = [{ printAtomicRMWOp(p, *this); }];
  let verifier = "return ::verify(*this);";
}

def LLVM_AtomicCmpXchgOp : LLVM_Op<"cmpxchg">,
    Arguments<(ins LLVM_Type:$ptr, LLVM_Type:$cmp, LLVM_Type:$val,
                   AtomicOrdering:$success_ordering,
                   AtomicOrdering:$failure_ordering)>,
    Results<(outs LLVM_Type:$res)> {
  let llvmBuilder = [{
    $res = builder.CreateAtomicCmpXchg($ptr, $cmp, $val,
                   getLLVMAtomicOrdering($success_ordering),
                   getLLVMAtomicOrdering($failure_ordering));
  }];
  let parser = [{ return parseAtomicCmpXchgOp(parser, result); }];
  let printer = [{ printAtomicCmpXchgOp(p, *this); }];
  let verifier = "return ::verify(*this);";
}

#endif // LLVMIR_OPS
