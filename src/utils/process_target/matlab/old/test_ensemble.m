%test different 4 situations with classifier

function [] = test_ensemble(model,stopped,good,aside,far)

evaluate_model(model,stopped);
evaluate_model(model,good);
evaluate_model(model,aside);
evaluate_model(model,far);